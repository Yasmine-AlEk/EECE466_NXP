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
    return_0:     float     # discounted return from step 0 — proxy for episode quality


class REINFORCETrainer:
    """
    Collects (log_prob, reward) pairs for one episode and applies the REINFORCE
    policy-gradient update at the end.

    Update rule (from the report, eq. 78):
        θ ← θ + α · Σ_t γ^t G_t · ∇_θ log π_θ(a_t | s_t)

    Return normalisation (enabled by default) subtracts the mean return and
    divides by std before computing the loss — this substantially reduces
    gradient variance without changing the optimal policy.
    """

    def __init__(
        self,
        policy,
        lr:                float = 1e-3,
        gamma:             float = 0.99,
        normalise_returns: bool  = True,
    ) -> None:
        self.policy            = policy
        self.optimizer         = optim.Adam(policy.parameters(), lr=lr)
        self.gamma             = gamma
        self.normalise_returns = normalise_returns

        self._log_probs: List[torch.Tensor] = []
        self._rewards:   List[float]        = []
        self._episode:   int                = 0

    # ------------------------------------------------------------------ #
    # Per-step accumulation                                                #
    # ------------------------------------------------------------------ #

    def store(self, log_prob: torch.Tensor, reward: float) -> None:
        self._log_probs.append(log_prob)
        self._rewards.append(reward)

    def steps(self) -> int:
        return len(self._rewards)

    def total_reward(self) -> float:
        return float(sum(self._rewards))

    # ------------------------------------------------------------------ #
    # End-of-episode update                                                #
    # ------------------------------------------------------------------ #

    def update(self) -> EpisodeStats:
        """
        Compute discounted returns, run one gradient step, clear the trajectory.
        Safe to call even when the trajectory is empty (returns zero-loss stats).
        """
        T = len(self._rewards)

        if T == 0:
            return EpisodeStats(
                episode=self._episode, steps=0,
                total_reward=0.0, loss=0.0, return_0=0.0,
            )

        # Monte-Carlo returns G_t = Σ_{k≥t} γ^{k-t} r_{k+1}
        returns = np.empty(T, dtype=np.float32)
        G = 0.0
        for t in reversed(range(T)):
            G = self._rewards[t] + self.gamma * G
            returns[t] = G

        returns_t = torch.as_tensor(returns, dtype=torch.float32)
        if self.normalise_returns and T > 1:
            returns_t = (returns_t - returns_t.mean()) / (returns_t.std() + 1e-8)

        # Policy loss: -Σ log π(a_t|s_t) · G_t
        loss = torch.stack(
            [-lp * Gt for lp, Gt in zip(self._log_probs, returns_t)]
        ).sum()

        self.optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(self.policy.parameters(), max_norm=1.0)
        self.optimizer.step()

        stats = EpisodeStats(
            episode=self._episode,
            steps=T,
            total_reward=float(sum(self._rewards)),
            loss=float(loss.item()),
            return_0=float(returns[0]),
        )

        self._episode += 1
        self._log_probs.clear()
        self._rewards.clear()
        return stats
