.. _peripheral_ras:

Bluetooth: Ranging Service (RAS)
################################

.. contents::
   :local:
   :depth: 2

The Peripheral Ranging Service (CGMS) sample demonstrates how to use the ranging service.

Requirements
************

The sample supports the following development kits:

.. table-from-sample-yaml::

.. include:: /includes/tfm.txt

Overview
********

TODO.

Building and running
********************

.. |sample path| replace:: :file:`samples/bluetooth/peripheral_ras`

.. include:: /includes/build_and_run_ns.txt

Testing
=======

TODO.

Dependencies
************

This sample uses the following Zephyr libraries:

* :file:`include/zephyr/types.h`
* :file:`include/errno.h`
* :file:`include/zephyr.h`
* :file:`include/sys/printk.h`
* :file:`include/sys/byteorder.h`

* :ref:`zephyr:bluetooth_api`:

* :file:`include/bluetooth/bluetooth.h`
* :file:`include/bluetooth/conn.h`
* :file:`include/bluetooth/uuid.h`
* :file:`include/bluetooth/gatt.h`
* :file:`include/bluetooth/services/ras.h`
