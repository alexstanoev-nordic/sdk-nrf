.. _central_rreq:

Bluetooth: Ranging Requestor (RREQ)
###################################

.. contents::
   :local:
   :depth: 2

The Ranging Requestor (RREQ) sample demonstrates how to use the ranging service (RAS) client.

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
