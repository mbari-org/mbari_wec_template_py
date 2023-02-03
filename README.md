# mbari_wec_template_py
Python template repo for external ROS2-enabled controller for MBARI's wave energy conversion buoy (sim or physical).

Please see [buoy_msgs/buoy_api_py](https://github.com/osrf/buoy_msgs/tree/main/buoy_api_py)
for controller [examples](https://github.com/osrf/buoy_msgs/tree/main/buoy_api_py/buoy_api/examples)

## Modify template for your package
Replace `mbari_wec_template_py` with your package name in

- package.xml
- setup.py
- setup.cfg
- launch/controller.launch.py

and rename two files/folders

- the empty file resource/mbari_wec_template_py
- the python package `mbari_wec_template_py` containing `controller.py`

Modify `setup.py` as desired and add any dependencies in `package.xml`.

## Implement Controller
Assuming you have followed the above and renamed the python package `mbari_wec_template_py` to your package name,
`<your_package_name>/controller.py` is stubbed out to implement your custom external controller.

You may also use `config/controller.yaml` for any policy parameters.
If you modify the controller node name, i.e. `super().__init__('controller')` and the `name` field of `Node` in the launch file,
please be sure to update the first line of the `config/controller.yaml` file and to use the same node name.

Also, if you choose a more specific name for your controller,
consider renaming:

- `launch/controller.launch.py`
- `config/controller.yaml`
- `<your_package_name>/controller.py`

and update:

- `console_scripts` in `setup.py`
- `executable` field of `Node` in `launch/controller.launch.py`

accordingly.
