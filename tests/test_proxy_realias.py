"""
Round 8.1 regression: multiros.core.MultirosGym must be the same
class object as uniros._proxy.GymProxy, so a fix landed in UniROS
also lands in multiros.
"""
from multiros.core import MultirosGym
from uniros._proxy import GymProxy
from uniros.core import uniros_gym


def test_multiros_gym_is_canonical_gym_proxy():
    assert MultirosGym is GymProxy


def test_multiros_gym_is_uniros_gym():
    assert MultirosGym is uniros_gym


def test_isinstance_via_multiros_alias():
    """An env created through any package should pass isinstance via MultirosGym."""
    # We can't easily instantiate without a real gym env, but identity
    # equality already guarantees isinstance behaviour for any
    # GymProxy instance.
    assert issubclass(MultirosGym, GymProxy)
    assert issubclass(GymProxy, MultirosGym)


def test_make_classmethod_present():
    assert hasattr(MultirosGym, "make")
    assert callable(MultirosGym.make)
