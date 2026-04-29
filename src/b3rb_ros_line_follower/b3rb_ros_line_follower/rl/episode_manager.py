from dataclasses import dataclass
from typing import List

import numpy as np
import torch
import torch.optim as optim


@dataclass
class EpisodeStats:
    episode:      int
    steps:        int
    total_reward: float
    loss:         float
    return_0:     float     # discounted return from step 0


class REINFORCETrainer:
    """
    Collects (log_prob, reward) pairs for one episode and applies the REINFORCE
    policy-gradient update at the end.

    Update rule:
        θ ← θ + α · Σ_t γ^t (G_t − b) · ∇_θ log π_θ(a_t | s_t)
        − α_ent · ∇_θ H(π_θ)          (entropy bonus)

    Two variance-reduction techniques are applied:
      1. Return normalisation: subtract episode mean and divide by std.
      2. Running-mean baseline: subtract an exponential moving average of
         episode returns across episodes, further reducing the gradient
         signal for 'average' episodes.

    An entropy bonus encourages exploration throughout training and prevents
    premature convergence to a near-zero residual (which would be useless).
    """

    def __init__(
        self,
        policy,
        lr:                float = 1e-3,
        gamma:             float = 0.99,
        normalise_returns: bool  = True,
        baseline_alpha:    float = 0.05,   # EMA coefficient for running baseline
        entropy_coeff:     float = 0.01,   # weight on entropy bonus
    ) -> None:
        self.policy            = policy
        self.optimizer         = optim.Adam(policy.parameters(), lr=lr)
        self.gamma             = gamma
        self.normalise_returns = normalise_returns
        self.baseline_alpha    = baseline_alpha
        self.entropy_coeff     = entropy_coeff

        self._log_probs: List[torch.Tensor] = []
        self._rewards:   List[float]        = []
        self._episode:   int                = 0

        # Running baseline: exponential moving average of G_0 across episodes
        self._running_baseline: float = 0.0
        self._baseline_ready:   bool  = False

    # ------------------------------------------------------------------ #

    def store(self, log_prob: torch.Tensor, reward: float) -> None:
        self._log_probs.append(log_prob)
        self._rewards.append(reward)

    def steps(self) -> int:
        return len(self._rewards)

    def total_reward(self) -> float:
        return float(sum(self._rewards))

    # ------------------------------------------------------------------ #

    def update(self) -> EpisodeStats:
        """
        Compute discounted returns, apply one gradient step, clear trajectory.
        Safe to call even when the trajectory is empty.
        """
        T = len(self._rewards)

        if T == 0:
            return EpisodeStats(
                episode=self._episode, steps=0,
                total_reward=0.0, loss=0.0, return_0=0.0,
            )

        # Monte-Carlo returns
        returns = np.empty(T, dtype=np.float32)
        G = 0.0
        for t in reversed(range(T)):
            G = self._rewards[t] + self.gamma * G
            returns[t] = G

        G0 = float(returns[0])

        # Update running baseline with this episode's G_0
        if not self._baseline_ready:
            self._running_baseline = G0
            self._baseline_ready   = True
        else:
            self._running_baseline = (
                (1.0 - self.baseline_alpha) * self._running_baseline
                + self.baseline_alpha * G0
            )

        device = next(self.policy.parameters()).device
        returns_t = torch.as_tensor(returns, dtype=torch.float32, device=device)

        # 1. Subtract running baseline before normalisation
        returns_t = returns_t - self._running_baseline

        # 2. Per-episode normalisation (further reduces variance)
        if self.normalise_returns and T > 1:
            returns_t = (returns_t - returns_t.mean()) / (returns_t.std() + 1e-8)

        # Policy loss: −Σ log π(a_t|s_t) · G_t
        policy_loss = torch.stack(
            [-lp * Gt for lp, Gt in zip(self._log_probs, returns_t)]
        ).sum()

        # Entropy bonus: encourage exploration, prevent collapsing to δ_rl ≈ 0
        entropy_loss = -self.entropy_coeff * self.policy.entropy()

        loss = policy_loss + entropy_loss

        self.optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(self.policy.parameters(), max_norm=1.0)
        self.optimizer.step()

        stats = EpisodeStats(
            episode=self._episode,
            steps=T,
            total_reward=float(sum(self._rewards)),
            loss=float(loss.item()),
            return_0=G0,
        )

        self._episode += 1
        self._log_probs.clear()
        self._rewards.clear()
        return stats
