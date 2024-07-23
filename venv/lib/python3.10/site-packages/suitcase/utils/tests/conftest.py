import bluesky
from bluesky.tests.conftest import RE  # noqa
from bluesky.plans import count
from bluesky.plan_stubs import trigger_and_read, configure
from ophyd.sim import SynGauss, SynAxis
import numpy as np

try:
    from ophyd.sim import DirectImage
except ImportError:
    from ophyd import Device, Component as Cpt
    from ophyd.sim import SynSignal

    class DirectImage(Device):
        img = Cpt(SynSignal, kind="hinted")

        def __init__(self, *args, func=None, **kwargs):
            super().__init__(*args, **kwargs)
            if func is not None:
                self.img._func = func

        def trigger(self):
            return self.img.trigger()


import event_model
import pytest
from .. import UnknownEventType
import warnings

if not hasattr(SynGauss, "configure"):

    class SynGauss(SynGauss):
        def configure(self, d):
            if d:
                raise ValueError
            return {}, {}


# This line is used to ignore the deprecation warning for bulk_events in tests
warnings.filterwarnings("ignore", message="The document type 'bulk_events'*")


_md = {"reason": "test", "user": "temp user", "beamline": "test_beamline"}


# Some useful plans for use in testing


def simple_plan(dets):
    """A simple plane which runs count with num=5"""
    md = {**_md, **{"test_plan_name": "simple_plan"}}
    yield from count(dets, num=5, md=md)


def multi_stream_one_descriptor_plan(dets):
    """A plan that has two streams but on descriptor per stream)"""
    md = {**_md, **{"test_plan_name": "multi_stream_one_descriptor_plan"}}

    @bluesky.preprocessors.baseline_decorator(dets)
    def _plan(dets):
        yield from count(dets, md=md)

    yield from _plan(dets)


def one_stream_multi_descriptors_plan(dets):
    '''A plan that has one stream but two descriptors per stream)'''
    md = {**_md, **{'test_plan_name': 'simple_plan'}}

    @bluesky.preprocessors.run_decorator(md=md)
    def _internal_plan(dets):
        yield from trigger_and_read(dets)
        for det in dets:
            yield from configure(det, {})
        yield from trigger_and_read(dets)

    yield from _internal_plan(dets)


def _make_single(ignore):
    if ignore:
        pytest.skip()
    motor = SynAxis(name="motor", labels={"motors"})
    det = SynGauss(
        "det", motor, "motor", center=0, Imax=1, sigma=1, labels={"detectors"}
    )
    return [det]


def _make_image(ignore):
    if ignore:
        pytest.skip()
    direct_img = DirectImage(
        func=lambda: np.array(np.ones((10, 10))), name="direct", labels={"detectors"}
    )

    return [direct_img]


def _make_image_list(ignore):
    if ignore:
        pytest.skip()
    direct_img_list = DirectImage(
        func=lambda: [[1] * 10] * 10, name="direct", labels={"detectors"}
    )
    direct_img_list.img.name = "direct_img_list"

    return [direct_img_list]


@pytest.fixture(
    params=[
        _make_single,
        _make_image,
        _make_image_list,
        lambda ignore: _make_image(ignore) + _make_image_list(ignore),
    ],
    scope="function",
)
def detector_list(request):  # noqa
    return request.param


@pytest.fixture(params=["event", "bulk_events", "event_page"], scope="function")
def event_type(request):
    def _event_type_func(ignore):
        if request.param in ignore:
            pytest.skip()
        return request.param

    return _event_type_func


@pytest.fixture(params=[simple_plan, multi_stream_one_descriptor_plan,
                        one_stream_multi_descriptors_plan],
                scope='function')
def plan_type(request):
    '''Returns a function that provides plan_types for testing.'''

    def _plan_type_func(skip_tests_with=None):
        '''Skips the current test or returns the plan_type in request.param for
        a number of test cases.

        skip_tests_with : list optional
            pytest.skip() any test with request.param in this list
        '''
        if skip_tests_with is None:
            skip_tests_with = []
        if request.param in skip_tests_with:
            pytest.skip()
        return request.param

    return _plan_type_func


@pytest.fixture(params=['test-', 'scan_{start[uid]}-'],
                scope='function')
def file_prefix_list(request):  # noqa
    '''Returns a function that provides file_prefixes for testing.
    '''

    def _file_prefix_list_func(skip_tests_with=None):
        '''Skips the current test or returns the file prefix in request.param for
        a number of test cases.

        skip_tests_with : list optional
            pytest.skip() any test with request.param in this list
        '''

        if skip_tests_with is None:
            skip_tests_with = []
        if request.param in skip_tests_with:
            pytest.skip()
        return request.param

    return _file_prefix_list_func


@pytest.fixture()
def generate_data(RE, detector_list, event_type):  # noqa
    '''A fixture that returns event data for a number of test cases.

    Returns a list of (name, doc) tuples for the plan passed in as an arg.

    Parameters
    ----------
    RE : object
        pytest fixture object imported from `bluesky.test.conftest`
    detector_list : list
        pytest fixture defined in `suitcase.utils.conftest` which returns a
        list of detectors
    event_type : list
        pytest fixture defined in `suitcase.utils.conftest` which returns a
        list of 'event_types'.
    '''

    def _generate_data_func(plan, skip_tests_with=None, md=None):
        '''Generates data to be used for testing of suitcase.*.export(..)
        functions

        Parameters
        ----------
        plan : the plan to use to generate the test data

        Returns
        -------
        collector : list
            A list of (name, doc) tuple pairs generated by the run engine.
        skip_tests_with : list, optional
            any test having request.param in this list will be skipped
        md : dict, optional
            metadata to be passed to the RunEngine
        '''
        if skip_tests_with is None:
            skip_tests_with = []
        if md is None:
            md = {}

        # define the output lists and an internal list.
        collector = []
        event_list = []

        # define the collector function depending on the event_type
        if event_type(skip_tests_with) == 'event':
            def collect(name, doc):
                collector.append((name, doc))
                if name == 'event':
                    event_list.append(doc)
        elif event_type(skip_tests_with) == 'event_page':
            def collect(name, doc):
                if name == 'event':
                    event_list.append(doc)
                elif name == 'stop':
                    collector.append(('event_page',
                                      event_model.pack_event_page(
                                          *event_list)))
                    collector.append((name, doc))
                else:
                    collector.append((name, doc))
        elif event_type(skip_tests_with) == 'bulk_events':
            def collect(name, doc):
                if name == 'event':
                    event_list.append(doc)
                elif name == 'stop':
                    collector.append(('bulk_events', {'primary': event_list}))
                    collector.append((name, doc))
                else:
                    collector.append((name, doc))
        else:
            raise UnknownEventType('Unknown event_type kwarg passed to '
                                   'suitcase.utils.events_data')

        # collect the documents
        RE(plan(detector_list(skip_tests_with)), collect, md=md)

        return collector

    return _generate_data_func


@pytest.fixture
def example_data(generate_data, plan_type):
    '''A fixture that returns event data for a number of test cases.

    Returns a function that returns a list of (name, doc) tuples for each of
    the plans in plan_type.

    .. note::

        It is recommended that you use this fixture for testing of
        ``suitcase-*`` export functions, for an example see
        ``suitcase-tiff.tests``. This will mean that future additions to the
        test suite here will be automatically applied to all ``suitcase-*``
        repos. Some important implementation notes:

        1. These fixtures are imported into other suitcase libraries via those
        libraries' ``conftest.py`` file. This is automatically set up by
        suitcases-cookiecutter, and no additional action is required.

        2. If any of the fixture parameters above are not valid for
        the suitcase you are designing and cause testing issues please skip
        them internally by adding them to the ``skip_tests_with`` kwarg list
        via the line
        ``collector = example_data(skip_tests_with=[param_to_ignore, ...])``.
        Take note ``param_to_ignore`` is the exact parameter, i.e. in case you
        want to ignore the tests against ``simple_plan`` in ``plan_type``
        ``param_to_ignore`` must actually be the function, not a string
        reference, which will need to be imported using:
        ``from suitcase.utils.tests.conftest import simple_plan``

    Parameters
    ----------
    generate_data : list
        pytest fixture defined in `suitcase.utils.conftest` which returns a
        function that accepts a plan as an argument and returns name, doc pairs
    plan_type : list
        pytest fixture defined in `suitcase.utils.conftest` which returns a
        list of 'plans' to test against.
    '''

    def _example_data_func(skip_tests_with=None, md=None):
        '''returns a list of (name, doc) tuples for a number of test cases

        skip_tests_with : list optional
            any test having request.param in this list will be skipped
        md : dict optional
            dict or dict-like object containing metadata to be added to the
            RunEngine
        '''

        return generate_data(
            plan_type(skip_tests_with), skip_tests_with=skip_tests_with, md=md)

    return _example_data_func
