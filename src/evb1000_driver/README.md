# What is this?
This is a driver node for the [TREK1000 Evaluation Kit](https://www.decawave.com/products/trek1000), DecaWave's Two-Way-Ranging RTLS IC Evaluation Kit and for the  [EVK1000 Evaluation Kit](https://www.decawave.com/products/evk1000-evaluation-kit).

The `evb1000_driver` package has ROS wrappers for the
  - `TREK1000_node` (UART `/dev/ttyACM0` to `/tagDistance_raw`, `/tagDistance_corrected` and `/anchorDistance`) running the `TREK1000 Evaluation Kit` firmware `v2.6`. The sensor data are published as `AnchorDistance.msg` and/or `TagDistance.msg` ROS messages.
  - `EVK1000_node` (UART `/dev/ttyACM0` to `/rangeMeasurement`) running the `EVK1000 Evaluation Kit` firmware `v3.05`.

# How to start it?

Start via `rosrun evb1000_driver EVK1000_node _device:="/dev/ttyACM0" ` or `rosrun evb1000_driver TREK1000_node _device:="/dev/ttyACM0" ` . The device /dev/ttyACM[Nr]
must be readable by the user that launch the node (add user to `dailout`).
If the device name is not /dev/ttyACM0 the name can be set via the argument „device“.

One can also use the launch file and the `default_config.yaml` to modify the behavior.

One can use dynamic reconfigure in `rqt` to dump the received data into a space separated file.
![alt text](/dyn_config.png "dyn configure")

## Links

[EVK1000](https://github.com/jungr-ait/snippets/blob/master/EVK1000_decawave_doc.md) <br/>
[TREK1000](https://github.com/jungr-ait/snippets/blob/master/TREK1000_decawave_doc.md)


## firmware

The proper firmware can be found in the firmware directory
