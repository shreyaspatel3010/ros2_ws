"""Concurrent velocity estimator: supervised deep learning trained alongside PPO
(Ji et al., "Concurrent Training of a Control Policy and a State Estimator", 2022).

The robot cannot measure its own pelvis velocity; the simulator knows it. A small network
inside the actor learns to predict it (supervised regression on the simulator's ground
truth, observation group "estimator_target") from the same IMU / joint history the policy
sees, and the policy receives that estimate as extra input. The estimate is detached for
the policy, so PPO does not distort the estimator, and the estimator is trained by its own
optimizer on every PPO update's rollouts.

Deployment is unchanged: the exported policy.pt / policy.onnx contain the estimator and
still map the normal observation vector to joint targets.

Used through the agent cfg (`class_name` strings resolved by RSL-RL):
    actor.class_name     = "hruh_lab.estimator:EstimatorActor"
    algorithm.class_name = "hruh_lab.estimator:EstimatorPPO"
"""
import copy

import torch
from torch import nn
from rsl_rl.algorithms import PPO
from rsl_rl.models import MLPModel
from rsl_rl.modules import MLP

TARGET_GROUP = "estimator_target"


class EstimatorActor(MLPModel):
    """MLP actor whose input is [observation, estimated pelvis velocity]."""

    estimate_dim = 3
    estimator_hidden_dims = (256, 128)

    def __init__(self, obs, obs_groups, obs_set, output_dim, **kwargs):
        super().__init__(obs, obs_groups, obs_set, output_dim, **kwargs)
        self.estimator = MLP(self.obs_dim, self.estimate_dim, list(self.estimator_hidden_dims),
                             kwargs.get("activation", "elu"))

    def _get_latent_dim(self):
        return self.obs_dim + self.estimate_dim

    def estimate(self, obs):
        """Estimated pelvis linear velocity (m/s, pelvis frame) - trainable path."""
        x = torch.cat([obs[group] for group in self.obs_groups], dim=-1)
        return self.estimator(self.obs_normalizer(x))

    def get_latent(self, obs, masks=None, hidden_state=None):
        x = self.obs_normalizer(torch.cat([obs[group] for group in self.obs_groups], dim=-1))
        return torch.cat((x, self.estimator(x).detach()), dim=-1)

    def as_jit(self):
        return _ExportedEstimatorActor(self)

    def as_onnx(self, verbose=False):
        return _ExportedEstimatorActor(self, onnx=True)


class _ExportedEstimatorActor(nn.Module):
    """Deterministic export: observation vector -> actions (estimator included)."""

    def __init__(self, model, onnx=False):
        super().__init__()
        self.obs_normalizer = copy.deepcopy(model.obs_normalizer)
        self.estimator = copy.deepcopy(model.estimator)
        self.mlp = copy.deepcopy(model.mlp)
        self.deterministic_output = (model.distribution.as_deterministic_output_module()
                                     if model.distribution is not None else nn.Identity())
        self.input_size = model.obs_dim
        self.onnx = onnx

    def forward(self, x):
        x = self.obs_normalizer(x)
        return self.deterministic_output(self.mlp(torch.cat((x, self.estimator(x)), dim=-1)))

    @torch.jit.export
    def reset(self):
        pass

    # ONNX exporter interface (same as RSL-RL's MLP export)
    def get_dummy_inputs(self):
        return (torch.zeros(1, self.input_size),)

    @property
    def input_names(self):
        return ["obs"]

    @property
    def output_names(self):
        return ["actions"]


class EstimatorPPO(PPO):
    """PPO plus one supervised estimator epoch per update on the same rollouts."""

    estimator_learning_rate = 1.0e-3

    def __init__(self, actor, critic, storage, *args, **kwargs):
        super().__init__(actor, critic, storage, *args, **kwargs)
        self.estimator_optimizer = torch.optim.Adam(actor.estimator.parameters(), lr=self.estimator_learning_rate)

    def update(self):
        losses, count = 0.0, 0
        # (PPO clears the rollout storage at the end of its update, so the estimator goes first)
        for batch in self.storage.mini_batch_generator(self.num_mini_batches, 1):
            target = batch.observations[TARGET_GROUP]
            loss = nn.functional.mse_loss(self.actor.estimate(batch.observations), target)
            self.estimator_optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(self.actor.estimator.parameters(), self.max_grad_norm)
            self.estimator_optimizer.step()
            losses += loss.item()
            count += 1
        result = super().update()
        result["estimator"] = losses / max(count, 1)   # mean squared velocity error, (m/s)^2
        return result
