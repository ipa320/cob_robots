## Component configuration

For each robot there is a folder called config, where you can find the
yaml files for each components of this robot. Here you can configure the
sensors and actors. You can find the information of this configuration
parameters in the following packages:

  -- ------------- -- -------------------------------------------------------------------------- --
     Component        Package                                                                    
     Base             [cob_base_drive_chain](https://index.ros.org/p/cob_base_drive_chain)       
     Torso            [schunk_powercube_chain](https://index.ros.org/p/schunk_powercube_chain)   
     Sdh              [schunk_sdh](https://index.ros.org/p/schunk_sdh)                           
     Lwa              [schunk_powercube_chain](https://index.ros.org/p/schunk_powercube_chain)   
     Tray             [schunk_powercube_chain](https://index.ros.org/p/schunk_powercube_chain)   
     Laser_front      [cob_sick_s300](https://index.ros.org/p/cob_sick_s300)                     
     Laser_rear       [cob_sick_s300](https://index.ros.org/p/cob_sick_s300)                     
     Head             [cob_head_axis](https://index.ros.org/p/cob_head_axis)                     
  -- ------------- -- -------------------------------------------------------------------------- --

## Urdf

The robot urdf is composed out of components defined in
[cob_description](https://index.ros.org/p/cob_description) and
[schunk_description](https://index.ros.org/p/schunk_description).

You can check your urdf settings with 

```
rosrun xacro xacro.py <<xacro_file>> > <<urdf_file>>
rosrun urdf_parser check_urdf <<urdf_file>>
```

 and
graphically with 

```
roslaunch urdf_tutorial display.launch gui:=True model:=<<urdf_file>>
```

 is for example
`cob3-3/urdf/cob3-3.urdf.xacro`.

## Calibration

Calibration data is stored in
[cob_calibration_data](https://index.ros.org/p/cob_calibration_data)
package.

The calibration of Care-O-bot have to be done in two steps, first manual
with the water balance you have to calibrate torso and tray, the second
step is automatic using the package [cob_calibration](cob_calibration) .
