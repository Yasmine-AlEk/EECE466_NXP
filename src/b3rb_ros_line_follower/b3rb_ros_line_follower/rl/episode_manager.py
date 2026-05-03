from dataclasses import dataclass
from typing import List

import numpy as np
import torch
import torch.nn.functional as F
import torch.optim as optim


@dataclass
class EpisodeStats:
    episode: int
    steps: int
    total_reward: float
    loss: float
    actor_loss: float
    critic_loss: float
    entropy: float
    return_0: float
    skipped_update: bool = False


class ActorCriticTrainer:
    """
    N-step actor-critic update.

    The actor uses advantage-weighted log probability.
    The critic learns an n-step bootstrapped return target.

    This is safer than pure Monte-Carlo REINFORCE because:
        - critic reduces variance
        - n-step target gives faster feedback
        - gradient clipping prevents one bad episode from destroying weights
        - failed episodes can be skipped by the runner if needed
    """

    def __init__(
        self,
        policy,
        lr: float = 3e-4,
        gamma: float = 0.99,
        n_step: int = 32,
        value_coef: float = 0.50,
        entropy_coef: float = 0.0015,
        grad_clip: float = 1.0,
        normalise_advantages: bool = True,
    ) -> None:
        self.policy = policy
        self.optimizer = optim.Adam(policy.parameters(), lr=lr)

        self.gamma = float(gamma)
        self.n_step = int(n_step)
        self.value_coef = float(value_coef)
        self.entropy_coef = float(entropy_coef)
        self.grad_clip = float(grad_clip)
        self.normalise_advantages = bool(normalise_advantages)

        self._log_probs: List[torch.Tensor] = []
        self._values: List[torch.Tensor] = []
        self._entropies: List[torch.Tensor] = []
        self._rewards: List[float] = []

        self._episode = 0

    def store(
        self,
        log_prob: torch.Tensor,
        value: torch.Tensor,
        entropy: torch.Tensor,
        reward: float,
    ) -> None:
        self._log_probs.append(log_prob)
        self._values.append(value)
        self._entropies.append(entropy)
        self._rewards.append(float(reward))

    def steps(self) -> int:
        return len(self._rewards)

    def total_reward(self) -> float:
        return float(sum(self._rewards))

    def clear(self, skipped_update: bool = False) -> EpisodeStats:
        stats = EpisodeStats(
            episode=self._episode,
            steps=len(self._rewards),
            total_reward=float(sum(self._rewards)),
            loss=0.0,
            actor_loss=0.0,
            critic_loss=0.0,
            entropy=0.0,
            return_0=self._discounted_return_0(),
            skipped_update=skipped_update,
        )

        self._episode += 1
        self._log_probs.clear()
        self._values.clear()
        self._entropies.clear()
        self._rewards.clear()

        return stats

    def _discounted_return_0(self) -> float:
        g = 0.0
        for r in reversed(self._rewards):
            g = float(r) + self.gamma * g
        return float(g)

    def update(self) -> EpisodeStats:
        T = len(self._rewards)

        if T == 0:
            return self.clear(skipped_update=False)

        device = next(self.policy.parameters()).device

        log_probs = torch.stack(self._log_probs).to(device)
        values = torch.stack(self._values).to(device).view(-1)
        entropies = torch.stack(self._entropies).to(device)

        rewards = np.asarray(self._rewards, dtype=np.float32)

        targets = []
        for t in range(T):
            g = 0.0
            power = 1.0

            horizon = min(self.n_step, T - t)

            for k in range(horizon):
                g += power * float(rewards[t + k])
                power *= self.gamma

            bootstrap_index = t + self.n_step
            if bootstrap_index < T:
                g += power * float(values[bootstrap_index].detach().item())

            targets.append(g)

        targets_t = torch.as_tensor(targets, dtype=torch.float32, device=device)

        advantages = targets_t - values.detach()

        if self.normalise_advantages and T > 1:
            adv_std = advantages.std()
            if torch.isfinite(adv_std) and float(adv_std.item()) > 1e-8:
                advantages = (advantages - advantages.mean()) / (adv_std + 1e-8)

        actor_loss = -(log_probs * advantages).mean()
        critic_loss = F.mse_loss(values, targets_t)
        entropy_mean = entropies.mean()

        loss = actor_loss + self.value_coef * critic_loss - self.entropy_coef * entropy_mean

        self.optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(self.policy.parameters(), self.grad_clip)
        self.optimizer.step()

        stats = EpisodeStats(
            episode=self._episode,
            steps=T,
            total_reward=float(sum(self._rewards)),
            loss=float(loss.item()),
            actor_loss=float(actor_loss.item()),
            critic_loss=float(critic_loss.item()),
            entropy=float(entropy_mean.item()),
            return_0=self._discounted_return_0(),
            skipped_update=False,
        )

        self._episode += 1
        self._log_probs.clear()
        self._values.clear()
        self._entropies.clear()
        self._rewards.clear()

        return stats
