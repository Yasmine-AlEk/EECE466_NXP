import numpy as np
import torch
import torch.nn as nn


class GaussianPolicy(nn.Module):
    """
    Stochastic Gaussian policy for REINFORCE.

    Architecture: Linear(state_dim, hidden) -> Tanh -> Linear(hidden, 1) -> Tanh
    The Tanh output is scaled by delta_max so the mean always lives in
    (-delta_max, +delta_max).  Log-std is a single learnable scalar.

    Usage
    -----
    training  : action, log_prob = policy.sample(state_np)
    deployment: action           = policy.act(state_np)
    """

    def __init__(
        self,
        state_dim:  int   = 4,
        hidden_dim: int   = 16,
        delta_max:  float = 0.30,
    ) -> None:
        super().__init__()
        self.delta_max = float(delta_max)

        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, 1),
            nn.Tanh(),
        )
        # Initialise log_std = -1  →  std ≈ 0.37 rad: moderate early exploration
        self.log_std = nn.Parameter(torch.tensor(-1.0))

    # ------------------------------------------------------------------ #

    def _mean(self, s: torch.Tensor) -> torch.Tensor:
        return self.net(s) * self.delta_max

    def sample(self, state_np: np.ndarray):
        """
        Stochastic forward pass (training).

        Returns
        -------
        action   : float  — sampled steering correction, clamped to ±delta_max
        log_prob : Tensor — stays in the computation graph for REINFORCE
        """
        s    = torch.as_tensor(state_np, dtype=torch.float32)
        mean = self._mean(s)
        std  = self.log_std.exp().clamp(min=1e-4)
        dist = torch.distributions.Normal(mean, std)
        raw  = dist.rsample()                                     # reparameterised
        log_prob = dist.log_prob(raw).squeeze()
        action   = float(raw.clamp(-self.delta_max, self.delta_max).item())
        return action, log_prob

    def act(self, state_np: np.ndarray) -> float:
        """Deterministic forward pass (deployment). No gradient tracking."""
        with torch.no_grad():
            s = torch.as_tensor(state_np, dtype=torch.float32)
            return float(self._mean(s).item())

    # ------------------------------------------------------------------ #

    def save(self, path: str) -> None:
        torch.save(self.state_dict(), path)

    def load(self, path: str) -> None:
        try:
            sd = torch.load(path, map_location="cpu", weights_only=True)
        except TypeError:
            # PyTorch < 2.0 does not have weights_only
            sd = torch.load(path, map_location="cpu")
        self.load_state_dict(sd)
        self.eval()
