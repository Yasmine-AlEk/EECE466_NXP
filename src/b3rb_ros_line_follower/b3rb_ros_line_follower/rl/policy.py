import math
from typing import Tuple

import numpy as np
import torch
import torch.nn as nn


class ActorCriticPolicy(nn.Module):
    """
    Actor-critic policy for residual control on top of MRAC.

    Action:
        d_steer: additive residual on MRAC steering command
        d_speed: additive residual on baseline speed command

    Important:
        The maximum RL contribution is NOT hard-coded here.
        It is still controlled by ROS parameters:
            delta_max, speed_up, speed_dn
    """

    def __init__(
        self,
        state_dim: int = 4,
        hidden_dim: int = 64,
        delta_max: float = 0.30,
        speed_up: float = 0.25,
        speed_dn: float = 0.05,
        device: str = "cpu",
    ) -> None:
        super().__init__()

        self.delta_max = float(delta_max)
        self.speed_up = float(speed_up)
        self.speed_dn = float(speed_dn)
        self.device = torch.device(device)

        self.shared = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
        )

        self.actor = nn.Sequential(
            nn.Linear(hidden_dim, 2),
            nn.Tanh(),
        )

        self.critic = nn.Linear(hidden_dim, 1)

        # Conservative exploration at the start.
        self.log_std = nn.Parameter(torch.tensor([-1.7, -1.7], dtype=torch.float32))

        for module in self.modules():
            if isinstance(module, nn.Linear):
                nn.init.orthogonal_(module.weight, gain=0.05)
                nn.init.zeros_(module.bias)

        self.to(self.device)

    def _features(self, state: torch.Tensor) -> torch.Tensor:
        return self.shared(state)

    def _scale_action(self, raw: torch.Tensor) -> torch.Tensor:
        """
        raw[:,0] -> steering residual in [-delta_max, +delta_max]
        raw[:,1] -> speed residual using piecewise scaling:
                    negative side limited by speed_dn
                    positive side limited by speed_up

        This keeps raw=0 mapped to d_speed=0, so the policy starts by
        preserving the MRAC/baseline behavior instead of always adding speed.
        """
        steer_raw = raw[..., 0:1]
        speed_raw = raw[..., 1:2]

        d_steer = steer_raw * self.delta_max
        d_speed = torch.where(
            speed_raw >= 0.0,
            speed_raw * self.speed_up,
            speed_raw * self.speed_dn,
        )

        return torch.cat([d_steer, d_speed], dim=-1)

    def forward(self, state: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        features = self._features(state)
        raw_mean = self.actor(features)
        value = self.critic(features).squeeze(-1)
        return raw_mean, value

    def sample(self, state_np: np.ndarray):
        """
        Training-time stochastic action.

        Returns:
            (d_steer, d_speed), log_prob, value, entropy
        """
        state = torch.as_tensor(state_np, dtype=torch.float32, device=self.device)

        raw_mean, value = self.forward(state)

        std = self.log_std.exp().clamp(min=1e-4, max=0.8)
        dist = torch.distributions.Normal(raw_mean, std)

        raw_action = dist.rsample()
        raw_action_clamped = raw_action.clamp(-1.0, 1.0)

        scaled_action = self._scale_action(raw_action_clamped)

        log_prob = dist.log_prob(raw_action).sum()
        entropy = dist.entropy().sum()

        d_steer = float(scaled_action[0].item())
        d_speed = float(scaled_action[1].item())

        return (d_steer, d_speed), log_prob, value, entropy

    def act(self, state_np: np.ndarray):
        """
        Deployment-time deterministic action.
        """
        with torch.no_grad():
            state = torch.as_tensor(state_np, dtype=torch.float32, device=self.device)
            raw_mean, _ = self.forward(state)
            scaled_action = self._scale_action(raw_mean)

            d_steer = float(scaled_action[0].item())
            d_speed = float(scaled_action[1].item())

            return d_steer, d_speed

    def save(self, path: str) -> None:
        torch.save(self.state_dict(), path)

    def load(self, path: str) -> bool:
        try:
            try:
                state_dict = torch.load(path, map_location=self.device, weights_only=True)
            except TypeError:
                state_dict = torch.load(path, map_location=self.device)

            self.load_state_dict(state_dict)
            self.to(self.device)
            return True

        except Exception as exc:
            import warnings
            warnings.warn(
                f"[RL policy] could not load checkpoint {path}; starting fresh. Error: {exc}"
            )
            return False
