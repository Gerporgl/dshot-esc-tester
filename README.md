Forked from JyeSmith/dshot-esc-tester

This is an ESP32 (from Espressif) arduino project that uses the dshot600 protocol to communicate with a blheli32 ESC and receive telemetry.

This fork from the original JyeSmith/dshot-esc-tester project is a bit simpler to use without all the LCD and other hardware required.

I added a PID controller using the RPM from the telemetry and made the dshot loop slightly faster and telemetry requests as well so that motor torque can increase very quickly using the PID feedback to compensate in output load differences on the motor.

In order to make the loop faster I relied only on timers, but at the end I could not make it much faster without having too many telemetry errors, so it is around 800us now, and also the telemetry speed is a lot slower than dshot so there is not much need to go too fast on the dshot side with a PID loop relying on telemetry feedback. The second core task is no longer used (somehow I couldn't use delayMicroseconds...), the main loop only gets the telemetry from the serial and reads the input from a pot to set the motor speed.

The PID can be disabled by just not connecting the telemetry pin (it times out after 25ms), or simply forcing the pid_on to be false in the code.
The current PID values were tested only briefly so far on a TMoror MN5008 without any real strong load on the motor except by hand to calibrate the PID values roughly, it seems to be working pretty well so far.

The purpose of that version is not to drive a drone or helicopter motor, but for general purpose brushless motor driving where you need to maintain a more precise constant RPM, or simply as a starting point when you want to drive a brushless motor with a potentiometer using dshot (the delay can be made shorter if you remove the smoothing algorithms, and even more if you don't need the PID)

Thanks to Jye Smith for putting together the only nice arduino based DSHOT esc example that is actually usable and fast. I'm hoping there will be many more libraries in the future to drive ESCs with digital protocols such as dshot to allow more precise motor control for general purpose use (and at low cost using generic hardware) and not just for drone motor control. At the time of this writing, there is really a lack of clean documentation about the DSHOT protocol in blheli at the electrical level as well as the list of special commands supported so far (1-47).