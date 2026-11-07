"""Numeric boundary checks for the offline dynamics contracts."""

from collections.abc import Mapping, Set
from itertools import islice
from math import isfinite, pi, remainder, tau
from numbers import Real


def finite_real(value, name):
    if isinstance(value, Real) and not isinstance(value, bool):
        try:
            result = float(value)
        except OverflowError:
            pass
        else:
            if isfinite(result):
                return result
    raise ValueError(f"{name} must be a finite real number")


def vector3(value, name):
    if isinstance(value, (str, bytes, Mapping, Set)):
        raise ValueError(f"{name} must contain three ordered components")
    try:
        components = tuple(islice(iter(value), 4))
    except TypeError as error:
        raise ValueError(f"{name} must contain three ordered components") from error
    if len(components) != 3:
        raise ValueError(f"{name} must contain exactly three components")
    return tuple(finite_real(component, name) for component in components)


def wrap_yaw(angle):
    wrapped = remainder(angle, tau)
    return -pi if wrapped == pi else wrapped
