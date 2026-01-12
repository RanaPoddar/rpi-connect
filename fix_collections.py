"""
Python 3.10+ compatibility fix for dronekit and other packages.
Patches collections module to restore deprecated aliases.
Must be imported before dronekit.
"""
import collections
import collections.abc

# Patch missing collections attributes that were moved to collections.abc
for name in ['MutableMapping', 'Mapping', 'MutableSequence', 'Sequence', 
             'MutableSet', 'Set', 'ItemsView', 'KeysView', 'ValuesView',
             'Iterable', 'Iterator', 'Callable', 'Container', 'Hashable',
             'Sized', 'Reversible', 'ByteString']:
    if not hasattr(collections, name):
        setattr(collections, name, getattr(collections.abc, name, None))
