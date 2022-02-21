"""Common assertions for convenience"""
from typing import Any, Union, Sequence, Collection

import numpy as np


def assert_type(value: object, type_: Any) -> None:
    """Asserts that inputs are of same type.

    :param value: Object to check
    :param type_: Type to be asserted
    :return:
    """
    assert isinstance(value, type_), f'Type {type(value)} provided when {type_} was expected.'


def assert_ndim(value: np.ndarray, ndim: int) -> None:
    """Asserts a specific number of dimensions for a numpy array.

    :param value: Numpy array to check
    :param ndim: Required number of dimensions
    :return:
    """
    assert value.ndim == ndim, f'Unexpected number of dimensions: {value.ndim} ({ndim} expected).'


def assert_len(value: Union[Sequence, Collection], len_: int):
    """Asserts a specific length for a sequence or a collection (e.g. a list).

    :param value: Sequence or collection to check
    :param len_: Required length
    :return:
    """
    assert len(value) == len_, f'Unexpected length: {len(value)} ({len_} expected).'


def assert_shape(value: np.ndarray, shape: tuple):
    """Asserts a specific shape for np.ndarray.

    :param value: Numpy array to check
    :param shape: Required shape
    :return:
    """
    assert value.shape == shape, f'Unexpected shape: {value.shape} ({shape} expected).'
