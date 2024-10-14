# Setup
## Steps to run - real environment
1. Must start 'cabo_state_node.py' first.
2. Must start 'cabo_voice_node.py' second.
3. Start other nodes in whatever order: 'cabo_motion_node.py', 'cabo_vision_node.py', and 'cabo_vision_display.py'.
4. After hearing "voice system initiated", you can try voice commands now.

## Silent test
1. Run 'cabo_voice_silent_test_node.py' instead of 'cabo_voice_node.py'.
2. Input "0" to simulate the completion of voice node initialization. 
3. See command mapping details in the code.

## Change vision input source
1. Change the last 3 lines of code in the 'cabo_vision_source.py' file, and re-compile.

## Vision input source
- If using pipeline source, must start vision node on the same computer of the depth camera.
- If using ROS2 source, any where is ok. However, this is node is not recommended when testing with the real robot, as the latency is significant.
- If using webcam source, anywhere is ok. Using this when testing without a robot, and for assisting the development of other nodes.

