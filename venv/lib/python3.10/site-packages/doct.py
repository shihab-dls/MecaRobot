"""
Copyright (c) 2015-, Brookhaven Science Associates, Brookhaven National
Laboratory. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the Brookhaven Science Associates, Brookhaven National
  Laboratory nor the names of its contributors may be used to endorse or promote
  products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
from collections.abc import Mapping
from functools import reduce
import time
import datetime

import humanize
from prettytable import PrettyTable


_HTML_TEMPLATE = """
<table>
{% for key, value in document | dictsort recursive %}
  <tr>
    <th> {{ key }} </th>
    <td>
      {% if value.items %}
        <table>
          {{ loop(value | dictsort) }}
        </table>
        {% else %}
          {% if key == 'time' %}
            {{ value | human_time }}
          {% else %}
            {{ value }}
          {% endif %}
        {% endif %}
    </td>
  </tr>
{% endfor %}
</table>
"""


class DocumentIsReadOnly(Exception):
    pass


class Document(dict):
    __slots__ = ('__dict__', )

    def __init__(self, name, *args, **kwargs):
        super(Document, self).__init__(*args, **kwargs)
        super(Document, self).__setitem__('_name', name)
        super(Document, self).__setattr__('__dict__', self)

    def __setattr__(self, key, value):
        raise DocumentIsReadOnly()

    def __setitem__(self, key, value):
        if isinstance(self.__dict__, Document):
            raise DocumentIsReadOnly('{}, {}'.format(key, value))
        else:
            return dict.__setitem__(self.__dict__, key, value)

    def __delattr__(self, key):
        raise DocumentIsReadOnly()

    def __delitem__(self, key):
        raise DocumentIsReadOnly()

    def update(self, *args, **kwargs):
        raise DocumentIsReadOnly()

    def pop(self, key, d=None):
        raise DocumentIsReadOnly()

    def __iter__(self):
        return (k for k in super(Document, self).__iter__()
                if not (isinstance(k, str) and k.startswith('_')))

    def items(self):
        return ((k, v) for k, v in super(Document, self).items()
                if not (isinstance(k, str) and k.startswith('_')))

    def values(self):
        return (v for k, v in super(Document, self).items()
                if not (isinstance(k, str) and k.startswith('_')))

    def keys(self):
        return (k for k in super(Document, self).keys()
                if not (isinstance(k, str) and k.startswith('_')))

    def __len__(self):
        return len(list(self.keys()))

    def __getstate__(self):
        return self._name, dict(self)

    def __setstate__(self, state):
        name, dd = state
        super(Document, self).__setattr__('__dict__', self)
        super(Document, self).__setitem__('_name', name)
        dict.update(self, dd)

    def _repr_html_(self):
        import jinja2
        env = jinja2.Environment()
        env.filters['human_time'] = pretty_print_time
        template = env.from_string(_HTML_TEMPLATE)
        return template.render(document=self)

    def __str__(self):
        return vstr(self)

    def to_name_dict_pair(self):
        """Convert to (name, dict) pair

        This can be used to safely mutate a Document::

           name, dd = doc.to_name_dict_pair()
           dd['new_key'] = 'aardvark'
           dd['run_start'] = dd['run_start']['uid']
           new_doc = Document(name, dd)

        Returns
        -------
        name : str
            Name of Document

        ret : dict
            Data payload of Document
        """
        ret = dict(self)
        ret.pop('_name', None)
        name = self._name
        return name, ret


def pretty_print_time(timestamp):
    # timestamp needs to be a float or fromtimestamp() will barf
    try:
        timestamp = float(timestamp)
    except (TypeError, ValueError):
        # not a float, assume it is already as pretty as it will get
        return timestamp
    dt = datetime.datetime.fromtimestamp(timestamp).isoformat()
    ago = humanize.naturaltime(time.time() - timestamp)
    return '{ago} ({date})'.format(ago=ago, date=dt)


def _format_dict(value, name_width, value_width, name, tabs=0):
    ret = ''
    for k, v in value.items():
        if isinstance(v, Mapping):
            ret += _format_dict(v, name_width, value_width, k, tabs=tabs+1)
        else:
            ret += ("\n%s%-{}s: %-{}s".format(
                name_width, value_width) % ('  '*tabs, k[:16], v))
    return ret


def _format_data_keys_dict(data_keys_dict):

    fields = reduce(set.union,
                    (set(v) for v in data_keys_dict.values()))
    fields = sorted(list(fields))
    table = PrettyTable(["data keys"] + list(fields))
    table.align["data keys"] = 'l'
    table.padding_width = 1
    for data_key, key_dict in sorted(data_keys_dict.items()):
        row = [data_key]
        for fld in fields:
            row.append(key_dict.get(fld, ''))
        table.add_row(row)
    return table


def vstr(doc, indent=0):
    """Recursive document walker and formatter

    Parameters
    ----------
    doc : Document
        Dict-like thing to format, must have `_name` key
    indent : int, optional
        The indentation level. Defaults to starting at 0 and adding one tab
        per recursion level
    """
    headings = [
        # characters recommended as headers by ReST docs
        '=', '-', '`', ':', '.', "'", '"', '~', '^', '_', '*', '+', '#',
        # all other valid header characters according to ReST docs
        '!', '$', '%', '&', '(', ')', ',', '/', ';', '<', '>', '?', '@',
        '[', '\\', ']', '{', '|', '}'
    ]
    name = doc['_name']

    ret = "\n%s\n%s" % (name, headings[indent]*len(name))

    documents = []
    name_width = 16
    value_width = 40
    for name, value in sorted(doc.items()):
        if name == 'descriptors':
            # this case is to deal with Headers from databroker
            for val in value:
                documents.append((name, val))
        elif name == 'data_keys':
            ret += "\n%s" % str(_format_data_keys_dict(value))
        elif isinstance(value, Mapping):
            if '_name' in value:
                documents.append((name, value))
            else:
                # format dicts reasonably
                ret += "\n%-{}s:".format(name_width) % (name)
                ret += _format_dict(value, name_width, value_width,
                                    name, tabs=1)
        else:
            ret += ("\n%-{}s: %-{}s".format(name_width, value_width) %
                    (name[:16], value))
    for name, value in documents:
        ret += "\n%s" % (vstr(value, indent+1))
        # ret += "\n"
    ret = ret.split('\n')
    ret = ["%s%s" % ('  '*indent, line) for line in ret]
    ret = "\n".join(ret)
    return ret


def ref_doc_to_uid(doc, field):
    """Convert a reference doc to a uid

    Given a Document, replace the given field (which must contain a
    Document) with the uid of that Document.

    Returns a new instance with the updated values

    Parameters
    ----------
    doc : Document
        The document to replace an entry in

    field : str
        The field to replace with the uid of it's contents
    """
    name, doc = doc.to_name_dict_pair()
    doc[field] = doc[field]['uid']
    return Document(name, doc)
