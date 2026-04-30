import numpy as np
import torch
import torch.nn as nn


class GaussianPolicy(nn.Module):
    """
    Two-output Gaussian policy: [δ_steer, δ_speed]

    δ_steer  ∈ [-steer_max,  +steer_max]   additive correction on MRAC turn cmd
    δ_speed  ∈ [-speed_dn,   +speed_up]    additive correction on baseline speed

    The speed output is intentionally asymmetric:
      - can increase speed by up to speed_up   (aggressive upward push)
      - can decrease speed by up to speed_dn   (small safety margin only)
    This asymmetry biases exploration toward going faster while still letting
    the policy brake slightly before tight corners.

    Architecture: 4 → 64 → 64 → 2  (Tanh activations)
    Two hidden layers with 64 units give enough capacity for the joint
    steer+speed policy without over-fitting.
    """

    def __init__(
        self,
        state_dim:  int   = 4,
        hidden_dim: int   = 64,      # wider than before to handle 2-output task
        delta_max:  float = 0.30,    # steering correction limit
        speed_up:   float = 0.25,    # max speed increase above baseline (m/s)
        speed_dn:   float = 0.05,    # max speed decrease below baseline (m/s)
        device: str = "cpu",
    ) -> None:
        super().__init__()
        self.delta_max = float(delta_max)
        self.speed_up  = float(speed_up)
        self.speed_dn  = float(speed_dn)
        self.device    = torch.device(device)

        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, 2),   # [steer_raw, speed_raw] both in (-1,1)
            nn.Tanh(),
        )

        # Two independent log-stds, both initialised to -1.5 (std ≈ 0.22)
        # so the policy starts conservative and explores from there.
        self.log_std = nn.Parameter(torch.tensor([-1.5, -1.5]))

        # Small orthogonal init → policy starts near zero residual,
        # letting MRAC do all the work on episode 1.
        for m in self.net.modules():
            if isinstance(m, nn.Linear):
                nn.init.orthogonal_(m.weight, gain=0.05)
                nn.init.zeros_(m.bias)

        self.to(self.device)

    # ------------------------------------------------------------------ #

    def _raw_mean(self, s: torch.Tensor) -> torch.Tensor:
        """Raw network output in (-1, 1) for each action dimension."""
        return self.net(s)   # shape (..., 2)

    def _scale_action(self, raw: torch.Tensor) -> torch.Tensor:
        """
        Scale the two raw outputs asymmetrically:
          steer:  raw * delta_max              → symmetric ±delta_max
          speed:  positive raw → +speed_up     → asymmetric [-speed_dn, +speed_up]
                  negative raw → -speed_dn
        """
        steer_raw = raw[..., 0:1]
        speed_raw = raw[..., 1:2]

        steer  = steer_raw * self.delta_max
        # Asymmetric scaling: remap (-1,1) → (-speed_dn, +speed_up)
        mid    = (self.speed_up - self.speed_dn) / 2.0
        half   = (self.speed_up + self.speed_dn) / 2.0
        speed  = speed_raw * half + mid

        return torch.cat([steer, speed], dim=-1)

    # ------------------------------------------------------------------ #

    def sample(self, state_np: np.ndarray):
        """
        Stochastic forward pass for training.

        Returns
        -------
        actions   : (δ_steer: float, δ_speed: float)
        log_probs : Tensor shape (2,) — stays in computation graph
        """
        s    = torch.as_tensor(state_np, dtype=torch.float32, device=self.device)
        mean = self._raw_mean(s)                               # (2,)
        std  = self.log_std.exp().clamp(min=1e-4, max=1.0)   # (2,)
        dist = torch.distributions.Normal(mean, std)
        raw  = dist.rsample()                                  # (2,)

        # Clamp to valid raw range before scaling
        raw_clamped = raw.clamp(-1.0, 1.0)
        actions = self._scale_action(raw_clamped)

        log_probs = dist.log_prob(raw)                         # (2,)
        log_prob_sum = log_probs.sum()                         # scalar Tensor

        d_steer = float(actions[0].item())
        d_speed = float(actions[1].item())
        return (d_steer, d_speed), log_prob_sum

    def act(self, state_np: np.ndarray):
        """Deterministic forward pass for deployment."""
        with torch.no_grad():
            s       = torch.as_tensor(state_np, dtype=torch.float32, device=self.device)
            raw     = self._raw_mean(s)
            actions = self._scale_action(raw)
            return float(actions[0].item()), float(actions[1].item())

    def entropy(self) -> torch.Tensor:
        """Sum of per-dimension Gaussian entropies."""
        return (self.log_std + 0.5 * (1.0 + torch.log(torch.tensor(2.0 * torch.pi)))).sum()

    # ------------------------------------------------------------------ #

    def save(self, path: str) -> None:
        torch.save(self.state_dict(), path)

    def load(self, path: str) -> None:
        """Load weights, gracefully ignoring shape mismatches (stale checkpoints)."""
        try:
            try:
                sd = torch.load(path, map_location=self.device, weights_only=True)
            except TypeError:
                sd = torch.load(path, map_location=self.device)
            self.load_state_dict(sd)
            self.to(self.device)
            self.eval()
        except (RuntimeError, Exception) as exc:
            import warnings
            warnings.warn(
                f"[RL policy] ignoring incompatible checkpoint at {path} "
                f"({exc}). Starting with fresh weights."
            )
