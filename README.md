# mbari_wec_template_py
Python template repo for external ROS2-enabled controller for MBARI's wave energy conversion buoy (sim or physical).
Please see [mbari_wec](https://github.com/osrf/mbari_wec) and associated
[documentation](https://osrf.github.io/mbari_wec). There is also a
[C++](https://github.com/mbari-org/mbari_wec_template_cpp) version of the template repository.

Please also see [mbari_wec_utils/buoy_api_py](https://github.com/osrf/mbari_wec_utils/tree/main/buoy_api_py)
for controller [examples](https://github.com/osrf/mbari_wec_utils/tree/main/buoy_api_py/buoy_api/examples)

## Tutorial
There is a [tutorial](https://osrf.github.io/mbari_wec/Tutorials/ROS2/PythonTemplate/) for a quick start
using this template. You may also refer to GitHub's
[template documentation](https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-repository-from-a-template)
for general help with template repositories.

## Modify template for your package
1. Click the green
button with the text `Use this template` and select `Create a new repository`

2. Next, set up the repository like you would any new GitHub repository choosing the owner,
repository name, public/private, etc.

3. Now that your new repository is set up, clone it to your local machine, make a branch, etc.  
   `$ git clone https://github.com/<owner>/<repo_name>.git`

You should now have a Python ROS 2 package with the following structure:

```
<repo_name>
    ├── config
    │   └── controller.yaml
    ├── launch
    │   └── controller.launch.py
    ├── LICENSE
    ├── mbari_wec_template_py
    │   ├── controller.py
    │   └── __init__.py
    ├── package.xml
    ├── README.md
    ├── resource
    │   └── mbari_wec_template_py
    ├── setup.cfg
    ├── setup.py
    └── test
        ├── test_copyright.py
        ├── test_flake8.py
        └── test_pep257.py
```

### Customizing the controller

You may also refer to the `README.md` in your newly cloned repository.

#### Modify template for your package
Replace `mbari_wec_template_py` with your package name and modify other fields as necessary in:

- package.xml (lines 4-8)
- setup.py (lines 7, 11, 22-25, 29)
- setup.cfg (lines 2, 4)  
    Update script_dir and install_scripts locations with your package name
- launch/controller.launch.py (lines 22, 34-35)
- config/controller.yaml (line 1)  
    Update first line with your controller name (same as node name in launch file)

and rename two files/folders

- the empty file `resource/mbari_wec_template_py`
- the Python package `mbari_wec_template_py` containing `controller.py`

resulting in the following folder structure:

```
repo_name
    ├── config
    │   └── controller.yaml
    ├── launch
    │   └── controller.launch.py
    ├── LICENSE
    ├── your_package_name
    │   ├── controller.py
    │   └── __init__.py
    ├── package.xml
    ├── README.md
    ├── resource
    │   └── your_package_name
    ├── setup.cfg
    ├── setup.py
    └── test
        ├── test_copyright.py
        ├── test_flake8.py
        └── test_pep257.py
```

Modify `setup.py` as desired and add any dependencies in `package.xml` following standard ROS 2
documentation.

---

## Implement Controller
Assuming you have followed the above and renamed the Python package `mbari_wec_template_py` to your package name,
`<your_package_name>/controller.py` is stubbed out to implement your custom external controller.
You may also use `config/controller.yaml` for any policy parameters.

### ControlPolicy

You may use the class `ControlPolicy` in `<your_package_name>/controller.py` to implement your controller.

- Set any configurable parameters in `__init__` on line 23
- Set any dependent variables in `update_params` on line 29
- Declare/get/update params in the `set_params` function of the `Controller` class on line 111
- Then, your control logic will go in the `target` function on line 36.
    Modify the input args as well as the return value as necessary

### Controller

The `Controller` class contains an instance of `ControlPolicy` as the member variable,
`self.policy`. The `self.policy.target` function may be called anywhere within the
`Controller` class. You may call it inside any of the data callbacks to enable feedback
control. Or, set up a loop in `main()` and run open-loop.

You may get feedback data from any of the buoy topics by simply creating a specific callback
listed below. For feedback data you'd like to use in another area of the class, feel free to
assign them to class variables.

(Delete any callbacks you don't need in the `Controller` class)

Available callback functions:

`/ahrs_data` &rarr; `def ahrs_callback(self, data):`  
`/battery_data` &rarr; `def battery_callback(self, data):`  
`/spring_data` &rarr; `def spring_callback(self, data):`  
`/power_data` &rarr; `def power_callback(self, data):`  
`/trefoil_data` &rarr; `def trefoil_callback(self, data):`  
`/powerbuoy_data` &rarr; `def powerbuoy_callback(self, data):`  

You may also send commands from within the `Controller` class:

`self.send_pump_command(duration_mins, blocking=False)`  
`self.send_valve_command(duration_sec, blocking=False)`  
`self.send_pc_wind_curr_command(wind_curr_amps, blocking=False)`  
`self.send_pc_bias_curr_command(bias_curr_amps, blocking=False)`  
`self.send_pc_scale_command(scale_factor, blocking=False)`  
`self.send_pc_retract_command(retract_factor, blocking=False)`  

---

## Example

An example using this interface can be found in the tutorial:
[Linear Damper Example (Python)](https://osrf.github.io/mbari_wec/Tutorials/ROS2/PythonLinearDamperExample.md)
