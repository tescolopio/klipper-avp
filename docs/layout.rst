Repository layout
=================

* ``avp/core``: measurement validation, adaptive grids, and clearance prediction.
* ``avp/klipper``: native Klipper command and probe integration.
* ``avp/history``: SQLite storage and retention.
* ``avp/analytics``: surface statistics and scan comparisons.
* ``tests/unit``: motion-independent algorithms and persistence tests.
* ``tests/simulation``: command tests with simulated toolhead/probe objects.
* ``tests/hardware``: reserved for supervised printer validation; currently
  contains no hardware tests and is excluded from CI.
* ``configs``: example printer configuration.
* ``docs``: project documentation.
* ``.github/workflows``: automated unit and simulation test runs.

The package root exposes ``load_config`` for Klipper's extras loader. Install
the complete ``avp`` directory as ``klippy/extras/avp`` so relative imports work
both as ``avp`` in development and as ``extras.avp`` inside Klipper. No package
installation or third-party Python dependencies are required.

Run all currently implemented tests from the repository root with::

    python3 -m unittest discover -s tests -v

Or run each suite separately::

    python3 -m unittest discover -s tests/unit -v
    python3 -m unittest discover -s tests/simulation -v

Hardware validation must remain supervised. Consult the README motion-safety
guidance before connecting a printer; simulation results do not prove physical
clearance or probing accuracy.
