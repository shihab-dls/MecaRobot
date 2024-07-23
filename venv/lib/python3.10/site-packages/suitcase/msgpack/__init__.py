# Suitcase subpackages should follow strict naming and interface conventions.
# The public API must include Serializer and should include export if it is
# intended to be user-facing. They should accept the parameters sketched here,
# but may also accpet additional required or optional keyword arguments, as
# needed.
import event_model
import msgpack
import msgpack_numpy
from pathlib import Path
import suitcase.utils
from ._version import get_versions

__version__ = get_versions()['version']
del get_versions


def export(gen, directory, file_prefix='{start[uid]}', **kwargs):
    """
    Export a stream of documents to msgpack.

    .. note::

        This can alternatively be used to write data to generic buffers rather
        than creating files on disk. See the documentation for the
        ``directory`` parameter below.

    Parameters
    ----------
    gen : generator
        expected to yield ``(name, document)`` pairs

    directory : string, Path or Manager.
        For basic uses, this should be the path to the output directory given
        as a string or Path object. Use an empty string ``''`` to place files
        in the current working directory.

        In advanced applications, this may direct the serialized output to a
        memory buffer, network socket, or other writable buffer. It should be
        an instance of ``suitcase.utils.MemoryBufferManager`` and
        ``suitcase.utils.MultiFileManager`` or any object implementing that
        interface. See the suitcase documentation at
        https://nsls-ii.github.io/suitcase for details.

    file_prefix : str, optional
        The first part of the filename of the generated output files. This
        string may include templates as in
        ``{start[proposal_id]}-{start[sample_name]}-``,
        which are populated from the RunStart document. The default value is
        ``{start[uid]}-`` which is guaranteed to be present and unique. A more
        descriptive value depends on the application and is therefore left to
        the user.

    **kwargs : kwargs
        Keyword arugments to be passed through to the underlying I/O library.

    Returns
    -------
    artifacts : dict
        dict mapping the 'labels' to lists of file names (or, in general,
        whatever resources are produced by the Manager)

    Examples
    --------

    Generate files with unique-identifier names in the current directory.

    >>> export(gen, '')

    Generate files with more readable metadata in the file names.

    >>> export(gen, '', '{start[plan_name]}-{start[motors]}-')

    Include the experiment's start time formatted as YYYY-MM-DD_HH-MM.

    >>> export(gen, '', '{start[time]:%Y-%m-%d_%H:%M}-')

    Place the files in a different directory, such as on a mounted USB stick.

    >>> export(gen, '/path/to/my_usb_stick')
    """
    with Serializer(directory, file_prefix, **kwargs) as serializer:
        for item in gen:
            serializer(*item)

    return serializer.artifacts


class Serializer(event_model.DocumentRouter):
    """
    Serialize a stream of documents to msgpack.

    .. note::

        This can alternatively be used to write data to generic buffers rather
        than creating files on disk. See the documentation for the
        ``directory`` parameter below.

    Parameters
    ----------
    directory : string, Path, or Manager
        For basic uses, this should be the path to the output directory given
        as a string or Path object. Use an empty string ``''`` to place files
        in the current working directory.

        In advanced applications, this may direct the serialized output to a
        memory buffer, network socket, or other writable buffer. It should be
        an instance of ``suitcase.utils.MemoryBufferManager`` and
        ``suitcase.utils.MultiFileManager`` or any object implementing that
        interface. See the suitcase documentation at
        https://nsls-ii.github.io/suitcase for details.

    file_prefix : str, optional
        The first part of the filename of the generated output files. This
        string may include templates as in
        ``{start[proposal_id]}-{start[sample_name]}-``,
        which are populated from the RunStart document. The default value is
        ``{start[uid]}-`` which is guaranteed to be present and unique. A more
        descriptive value depends on the application and is therefore left to
        the user.

    flush : boolean
        Flush the file to disk after each document. As a consequence, writing
        the full document stream is slower but each document is immediately
        available for reading. False by default.

    **kwargs : kwargs
        Keyword arugments to be passed through to the underlying I/O library.

    Attributes
    ----------
    artifacts
        dict mapping the 'labels' to lists of file names (or, in general,
        whatever resources are produced by the Manager)
    """
    def __init__(self, directory, file_prefix='{start[uid]}', flush=False,
                 **kwargs):

        self._file_prefix = file_prefix
        self._flush = flush
        self._kwargs = kwargs
        self._templated_file_prefix = ''  # set when we get a 'start' document

        if isinstance(directory, (str, Path)):
            # The user has given us a filepath; they want files.
            # Set up a MultiFileManager for them.
            self._manager = suitcase.utils.MultiFileManager(directory)
        else:
            # The user has given us their own Manager instance. Use that.
            self._manager = directory

        self._buffer = None
        self._closed = False

    @property
    def artifacts(self):
        return self._manager.artifacts

    def close(self):
        """
        Close all of the resources (e.g. files) allocated.
        """
        if not self._closed:
            self._closed = True
            self._manager.close()

    def __enter__(self):
        return self

    def __exit__(self, *exception_details):
        self.close()

    def start(self, doc):
        # Fill in the file_prefix with the contents of the RunStart document.
        # As in, '{tart[uid]}' -> 'c1790369-e4b2-46c7-a294-7abfa239691a'
        # or 'my-data-from-{plan-name}' -> 'my-data-from-scan'
        filename = f'{self._file_prefix.format(start=doc)}.msgpack'
        self._buffer = self._manager.open('all', filename, 'xb')
        self._buffer.write(_encode(('start', doc), **self._kwargs))
        if self._flush:
            self._buffer.flush()

    def descriptor(self, doc):
        self._buffer.write(_encode(('descriptor', doc), **self._kwargs))
        if self._flush:
            self._buffer.flush()

    def event_page(self, doc):
        self._buffer.write(_encode(('event_page', doc), **self._kwargs))
        if self._flush:
            self._buffer.flush()

    def stop(self, doc):
        self._buffer.write(_encode(('stop', doc), **self._kwargs))
        self.close()

    # This suitcase can be used to store "unfilled" Events, Events that
    # reference external files using Datum[Page] and Resource documents.
    # Not all suitcases do this.

    def datum_page(self, doc):
        self._buffer.write(_encode(('datum_page', doc), **self._kwargs))
        if self._flush:
            self._buffer.flush()

    def resource(self, doc):
        self._buffer.write(_encode(('resource', doc), **self._kwargs))
        if self._flush:
            self._buffer.flush()


def _encode(obj, default=msgpack_numpy.encode, use_bin_type=True, **kwargs):
    "Encode as msgpack using numpy-aware encoder."
    # See https://github.com/msgpack/msgpack-python#string-and-binary-type
    # for more on use_bin_type.
    return msgpack.packb(obj, default=default, use_bin_type=use_bin_type,
                         **kwargs)
