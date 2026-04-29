import numpy as np
import torch
import torch.nn as nn


class GaussianPolicy(nn.Module):
    """
    Stochastic Gaussian policy for REINFORCE.

    Architecture: Linear(state_dim, hidden) -> Tanh -> Linear(hidden, hidden) -> Tanh
                  -> Linear(hidden, 1) -> Tanh -> scaled by delta_max

    Two hidden layers give the policy enough capacity to learn velocity-
    dependent steering corrections without over-fitting on a shallow track.
    Log-std is a learnable scalar, initialised to -1 (std ≈ 0.37).

    Usage
    -----
    training  : action, log_prob = policy.sample(state_np)
    deployment: action           = policy.act(state_np)
    """

    def __init__(
        self,
        state_dim:  int   = 4,
        hidden_dim: int   = 32,
        delta_max:  float = 0.30,
        device: str = "cpu",
    ) -> None:
        super().__init__()
        self.delta_max = float(delta_max)
        self.device = torch.device(device)

        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, 1),
            nn.Tanh(),
        )
        # log_std = -1 → std ≈ 0.37 at the start; will be learned
        self.log_std = nn.Parameter(torch.tensor(-1.0))

        # Initialise weights with small values so the policy starts near
        # zero residual (letting MRAC do the work at the beginning).
        for m in self.net.modules():
            if isinstance(m, nn.Linear):
                nn.init.orthogonal_(m.weight, gain=0.1)
                nn.init.zeros_(m.bias)

        self.to(self.device)

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
        s    = torch.as_tensor(state_np, dtype=torch.float32, device=self.device)
        mean = self._mean(s)
        std  = self.log_std.exp().clamp(min=1e-4, max=1.0)
        dist = torch.distributions.Normal(mean, std)
        raw  = dist.rsample()
        log_prob = dist.log_prob(raw).squeeze()
        action   = float(raw.clamp(-self.delta_max, self.delta_max).item())
        return action, log_prob

    def act(self, state_np: np.ndarray) -> float:
        """Deterministic forward pass (deployment). No gradient tracking."""
        with torch.no_grad():
            s = torch.as_tensor(state_np, dtype=torch.float32, device=self.device)
            return float(self._mean(s).item())

    def entropy(self) -> torch.Tensor:
        """Gaussian entropy — used as a training regulariser."""
        return self.log_std + 0.5 * (1.0 + torch.log(torch.tensor(2.0 * torch.pi)))

    # ------------------------------------------------------------------ #

    def save(self, path: str) -> None:
        torch.save(self.state_dict(), path)

    def load(self, path: str) -> None:
        try:
            sd = torch.load(path, map_location=self.device, weights_only=True)
        except TypeError:
            sd = torch.load(path, map_location=self.device)
        self.load_state_dict(sd)
        self.to(self.device)
        self.eval()
