import pytest

from suitcase.utils import (
    MultiFileManager, MemoryBuffersManager, SuitcaseUtilsValueError,
    SuitcaseUtilsTypeError, ModeError)
pytest


def test_multifile_basic_operation(tmp_path):
    manager = MultiFileManager(tmp_path)
    f = manager.open('thing', 'stuff', 'x')
    name1 = tmp_path / 'stuff'
    name2 = manager.reserve_name('thing', 'stuff2')

    # Check that name clashes are not allowed.
    with pytest.raises(SuitcaseUtilsValueError):
        manager.open('thing', 'stuff', 'x')
    with pytest.raises(SuitcaseUtilsValueError):
        manager.reserve_name('thing', 'stuff')

    # Check that abs paths are not allowed.
    with pytest.raises(SuitcaseUtilsValueError):
        manager.open('thing', '/stuff', 'x')

    # Check that only certain modes are allowed.
    for mode in ('x', 'xt', 'xb'):
        manager.open('mode_tests', f'stuff-{mode}', mode)
    for mode in ('w', 'wt', 'wb'):
        with pytest.raises(ModeError):
            manager.open('mode_tests', f'stuff-{mode}', mode)

    f.write('test')
    assert not f.closed
    manager.close()
    assert f.closed
    assert 4 == manager.estimated_sizes['stuff']
    with open(name1) as f:
        actual = f.read()
    assert actual == 'test'
    assert [name1, name2] == manager.artifacts['thing']
    assert 2 == len(manager.get_artifacts('thing'))
    assert 'stuff' == manager.get_artifacts('thing')[0]['postfix']

    # Test append.
    manager = MultiFileManager(tmp_path)
    with manager.open('test_append', 'test_append', 'x') as f:
        f.write('line1\n')
    manager = MultiFileManager(tmp_path, allowed_modes=('a',))
    with manager.open('test_append', 'test_append', 'a') as f:
        f.write('line2\n')
    with open(tmp_path / 'test_append') as f:
        assert f.read().splitlines() == ['line1', 'line2']


def test_memory_buffers_basic_operation():
    manager = MemoryBuffersManager()
    f = manager.open('thing', 'stuff', 'x')

    # Check that reserve_name fails explicitly on MemoryBuffersManager.
    with pytest.raises(SuitcaseUtilsTypeError):
        manager.reserve_name('thing', 'stuff')

    # Check that name clashes are not allowed.
    with pytest.raises(SuitcaseUtilsValueError):
        manager.open('thing', 'stuff', 'x')

    # Check that abs paths are not allowed.
    with pytest.raises(SuitcaseUtilsValueError):
        manager.open('thing', '/stuff', 'x')

    # Check that only certain modes are allowed.
    for mode in ('x', 'xt', 'xb'):
        manager.open('mode_tests', f'stuff-{mode}', mode)
    for mode in ('w', 'wt', 'wb'):
        with pytest.raises(ModeError):
            manager.open('mode_tests', f'stuff-{mode}', mode)

    f.write('test')
    assert not f.closed
    manager.close()
    assert not f.closed  # Close is a no-op on Persistent{String|Bytes}IO.
    assert 4 == manager.estimated_sizes['stuff']
    f.seek(0)
    actual = f.read()
    assert actual == 'test'
    assert [f] == manager.artifacts['thing']
    assert 1 == len(manager.get_artifacts('thing'))
    assert 'stuff' == manager.get_artifacts('thing')[0]['postfix']


def test_fixture(example_data):
    "Just exercise the example_data test fixture."
    assert type(example_data()) is list


def test_fixture_with_skip_tests_with(example_data):
    """
    Exercise the example_data 'skip_tests_with' argument that skips
    tests having the specified test parameters.
    """
    assert type(example_data(skip_tests_with=['bulk_events'])) is list


def test_fixture_with_md(example_data):
    "Exercise the example_data 'md' argument that is passed to the RunEngine."
    document_stream = example_data(md={'user': 'Dan'})
    assert type(document_stream) is list

    name, document = document_stream[0]
    assert name == 'start'
    assert document['md']['user'] == 'Dan'
