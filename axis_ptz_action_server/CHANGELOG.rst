^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package axis_ptz_action_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#3 <https://github.com/clearpathrobotics/ptz_action_server/issues/3>`_ from clearpathrobotics/ONAV-1687
  rename file and add environment variables to set ptz limits
* fix endline
* restore limits_dome.yaml file as an example.
* Set the is_moving flag _after\_ we send the command to the camera hardware, just to prevent possible issues with feedback kicking out early
* Add an additional check to make sure the camera hasn't stopped moving. Make setting self.is_moving atomic with a setter function and a lock
* Add a new config file for dome cameras mounted upside-down
* `self.ptz_state.pan` and `.tilt` are already in radians; don't covnert them again
* rename file and add environment variables to set ptz limits
* Contributors: Chris Iverach-Brereton, José Mastrangelo, jmastrangelo-cpr

0.1.4 (2023-08-18)
------------------
* Add the ability to publish the Axis camera's joint states
* Contributors: Chris Iverach-Brereton

0.1.3 (2023-08-04)
------------------

0.1.2 (2023-05-05)
------------------
* Re-send the axis command if the camera isn't moving after 5s
* Add some additional logging, handle camera disconnections better
* Abort the action in-progress if the Axis camera stops publishing state messages
* Use format strings instead of .format
* Contributors: Chris Iverach-Brereton

0.1.1 (2023-04-11)
------------------
* Add limits files for the standard dome camera & Q62 series. Add a new launch argument to specify the limits file
* Contributors: Chris Iverach-Brereton

0.1.0 (2023-03-27)
------------------
* Initial release
* Contributors: Chris Iverach-Brereton
