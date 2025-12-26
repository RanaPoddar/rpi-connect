"""
Compatibility fix for Python 3.13+
Dronekit uses deprecated collections.MutableMapping
"""
import collections
import collections.abc

# Restore deprecated classes for backwards compatibility
for name in ['MutableMapping', 'MutableSet', 'MutableSequence']:
    if not hasattr(collections, name):
        setattr(collections, name, getattr(collections.abc, name))
