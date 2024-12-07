import numpy as np
import pytest
from pydrake.geometry.optimization import HPolyhedron


def test_environment_initialization():
    """Test basic environment setup."""
    env = Environment()

    # Test that environment components exist
    assert env.obstacles is not None
    assert env.vertices is not None
    assert env.x_min is not None
    assert env.x_max is not None
    assert env.x_start is not None
    assert env.x_goal is not None
    assert env.regions is not None

    # Test dimensions
    assert len(env.obstacles) == 6
    assert len(env.vertices) == 12
    assert len(env.regions) == 12

    # Test boundary conditions
    assert np.all(env.x_min < env.x_max)
    assert np.all(env.x_start >= env.x_min) and np.all(env.x_start <= env.x_max)
    assert np.all(env.x_goal >= env.x_min) and np.all(env.x_goal <= env.x_max)


def test_make_hpolytope():
    """Test HPolyhedron creation."""
    vertices = np.array([[0, 0], [1, 0], [1, 1], [0, 1]])
    env = Environment()
    polytope = env._make_hpolytope(vertices)

    assert isinstance(polytope, HPolyhedron)
