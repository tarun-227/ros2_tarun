4-Wheeled Robot Simulation in ROS 2 + Gazebo (Intern Project)
=============================================================

This README documents the complete steps and structure used to simulate a 4-wheeled robot with obstacle detection in **ROS 2 Humble** and **Gazebo 11**. The goal was to create a differential drive robot model, visualize it in RViz2, simulate it in Gazebo, and use a LIDAR sensor to stop the robot if an obstacle is nearby.

✅ What We Did (Step-by-Step)
----------------------------

### 1\. **Workspace Setup**

We created a new workspace named robot.

Plain textANTLR4BashCC#CSSCoffeeScriptCMakeDartDjangoDockerEJSErlangGitGoGraphQLGroovyHTMLJavaJavaScriptJSONJSXKotlinLaTeXLessLuaMakefileMarkdownMATLABMarkupObjective-CPerlPHPPowerShell.propertiesProtocol BuffersPythonRRubySass (Sass)Sass (Scss)SchemeSQLShellSwiftSVGTSXTypeScriptWebAssemblyYAMLXML`   mkdir -p ~/robot/src  cd ~/robot  colcon build  source install/setup.bash   `

### 2\. **bot\_description Package**

This package contains:

*   The robot's **XACRO** model (robot.xacro)
    
*   LIDAR sensor plugin and differential drive plugin
    
*   Launch files for RViz and Gazebo
    

Command to create:

Plain textANTLR4BashCC#CSSCoffeeScriptCMakeDartDjangoDockerEJSErlangGitGoGraphQLGroovyHTMLJavaJavaScriptJSONJSXKotlinLaTeXLessLuaMakefileMarkdownMATLABMarkupObjective-CPerlPHPPowerShell.propertiesProtocol BuffersPythonRRubySass (Sass)Sass (Scss)SchemeSQLShellSwiftSVGTSXTypeScriptWebAssemblyYAMLXML`   cd ~/robot/src  ros2 pkg create bot_description --build-type ament_python   `

### 3\. **URDF/XACRO Robot Model**

The robot.xacro defines:

*   A base link (base)
    
*   4 wheels (front\_left, front\_right, rear\_left, rear\_right) with continuous joints
    
*   A LIDAR link mounted on top of the base
    
*   Two major plugins:
    
    *   libgazebo\_ros\_ray\_sensor.so: Publishes LIDAR data
        
    *   libgazebo\_ros\_diff\_drive.so: Accepts /cmd\_vel to move robot
        

> **Note**: The wheel joints are configured as continuous joints with a defined axis.

### 4\. **Launch Files**

#### a. view\_model.launch.py

Launches RViz2 and robot\_state\_publisher to visualize the robot from URDF.

#### b. spawn\_robot.launch.py

Spawns the robot in Gazebo and launches robot\_state\_publisher.

#### c. gazebo\_with\_obstacle.launch.py

Master launch file that:

*   Launches Gazebo
    
*   Spawns the robot
    
*   Runs the obstacle stop behavior
    

### 5\. **obstacle\_stop Package**

Custom ROS 2 package to control robot based on LIDAR input.

Command to create:

Plain textANTLR4BashCC#CSSCoffeeScriptCMakeDartDjangoDockerEJSErlangGitGoGraphQLGroovyHTMLJavaJavaScriptJSONJSXKotlinLaTeXLessLuaMakefileMarkdownMATLABMarkupObjective-CPerlPHPPowerShell.propertiesProtocol BuffersPythonRRubySass (Sass)Sass (Scss)SchemeSQLShellSwiftSVGTSXTypeScriptWebAssemblyYAMLXML`   cd ~/robot/src  ros2 pkg create obstacle_stop --build-type ament_python --dependencies rclpy sensor_msgs geometry_msgs   `

It contains a node:

*   Subscribes to /scan topic
    
*   Publishes /cmd\_vel to move or stop based on obstacle proximity
    

> If the distance is less than 0.5m, the robot stops.

### 6\. **TF (Transform Tree)**

The transform tree defines the relationships between all the robot's parts. It is automatically published by robot\_state\_publisher based on the URDF.

TF Tree looks like:

Plain textANTLR4BashCC#CSSCoffeeScriptCMakeDartDjangoDockerEJSErlangGitGoGraphQLGroovyHTMLJavaJavaScriptJSONJSXKotlinLaTeXLessLuaMakefileMarkdownMATLABMarkupObjective-CPerlPHPPowerShell.propertiesProtocol BuffersPythonRRubySass (Sass)Sass (Scss)SchemeSQLShellSwiftSVGTSXTypeScriptWebAssemblyYAMLXML`   base  ├── front_left  ├── front_right  ├── rear_left  ├── rear_right  └── lidar   `

You can visualize the tree:

Plain textANTLR4BashCC#CSSCoffeeScriptCMakeDartDjangoDockerEJSErlangGitGoGraphQLGroovyHTMLJavaJavaScriptJSONJSXKotlinLaTeXLessLuaMakefileMarkdownMATLABMarkupObjective-CPerlPHPPowerShell.propertiesProtocol BuffersPythonRRubySass (Sass)Sass (Scss)SchemeSQLShellSwiftSVGTSXTypeScriptWebAssemblyYAMLXML`   ros2 run tf2_tools view_frames   `

🤖 URDF/XACRO Model Description
-------------------------------

The robot model is created using a .xacro file (robot.xacro) which is an XML macro language used to simplify URDF (Unified Robot Description Format) files. Xacro allows the use of variables, macros, and cleaner reuse of structures for robots with repetitive parts like wheels or arms.

### 🧩 Main Components in the URDF

#### 1\. **Parameters and Colors**

Plain textANTLR4BashCC#CSSCoffeeScriptCMakeDartDjangoDockerEJSErlangGitGoGraphQLGroovyHTMLJavaJavaScriptJSONJSXKotlinLaTeXLessLuaMakefileMarkdownMATLABMarkupObjective-CPerlPHPPowerShell.propertiesProtocol BuffersPythonRRubySass (Sass)Sass (Scss)SchemeSQLShellSwiftSVGTSXTypeScriptWebAssemblyYAMLXML

We define configurable constants for dimensions, and reusable color definitions using .

#### 2\. **Base Link (base)**

Plain textANTLR4BashCC#CSSCoffeeScriptCMakeDartDjangoDockerEJSErlangGitGoGraphQLGroovyHTMLJavaJavaScriptJSONJSXKotlinLaTeXLessLuaMakefileMarkdownMATLABMarkupObjective-CPerlPHPPowerShell.propertiesProtocol BuffersPythonRRubySass (Sass)Sass (Scss)SchemeSQLShellSwiftSVGTSXTypeScriptWebAssemblyYAMLXML`   ...   `

This is the main body of the robot. It contains:

*   A block (for appearance in RViz/Gazebo)
    
*   A block (for physics engine)
    
*   An block (for simulation dynamics like mass and inertia)
    

#### 3\. **Wheel Macro**

Plain textANTLR4BashCC#CSSCoffeeScriptCMakeDartDjangoDockerEJSErlangGitGoGraphQLGroovyHTMLJavaJavaScriptJSONJSXKotlinLaTeXLessLuaMakefileMarkdownMATLABMarkupObjective-CPerlPHPPowerShell.propertiesProtocol BuffersPythonRRubySass (Sass)Sass (Scss)SchemeSQLShellSwiftSVGTSXTypeScriptWebAssemblyYAMLXML`   ...   `

We define a macro to create a wheel link and joint. It:

*   Uses a cylinder shape rotated by 90° (rpy="0 1.57 0")
    
*   Attaches to the base using a **continuous joint**, allowing rotation.
    
*   Takes in name, x, and y parameters for position.
    

#### 4\. **Wheel Instances**

Plain textANTLR4BashCC#CSSCoffeeScriptCMakeDartDjangoDockerEJSErlangGitGoGraphQLGroovyHTMLJavaJavaScriptJSONJSXKotlinLaTeXLessLuaMakefileMarkdownMATLABMarkupObjective-CPerlPHPPowerShell.propertiesProtocol BuffersPythonRRubySass (Sass)Sass (Scss)SchemeSQLShellSwiftSVGTSXTypeScriptWebAssemblyYAMLXML

This calls the macro 4 times to define 4 wheels.

#### 5\. **LIDAR Sensor**

Plain textANTLR4BashCC#CSSCoffeeScriptCMakeDartDjangoDockerEJSErlangGitGoGraphQLGroovyHTMLJavaJavaScriptJSONJSXKotlinLaTeXLessLuaMakefileMarkdownMATLABMarkupObjective-CPerlPHPPowerShell.propertiesProtocol BuffersPythonRRubySass (Sass)Sass (Scss)SchemeSQLShellSwiftSVGTSXTypeScriptWebAssemblyYAMLXML`   ...  ...   `

LIDAR is added as a cylinder link mounted above the robot's base.

#### 6\. **LIDAR Plugin**

Plain textANTLR4BashCC#CSSCoffeeScriptCMakeDartDjangoDockerEJSErlangGitGoGraphQLGroovyHTMLJavaJavaScriptJSONJSXKotlinLaTeXLessLuaMakefileMarkdownMATLABMarkupObjective-CPerlPHPPowerShell.propertiesProtocol BuffersPythonRRubySass (Sass)Sass (Scss)SchemeSQLShellSwiftSVGTSXTypeScriptWebAssemblyYAMLXML  

Simulates a LIDAR sensor in Gazebo using libgazebo\_ros\_ray\_sensor.so. It:

*   Emits 360 rays
    
*   Publishes to /scan
    
*   Supports noise and range configuration
    

#### 7\. **Differential Drive Plugin**

Plain textANTLR4BashCC#CSSCoffeeScriptCMakeDartDjangoDockerEJSErlangGitGoGraphQLGroovyHTMLJavaJavaScriptJSONJSXKotlinLaTeXLessLuaMakefileMarkdownMATLABMarkupObjective-CPerlPHPPowerShell.propertiesProtocol BuffersPythonRRubySass (Sass)Sass (Scss)SchemeSQLShellSwiftSVGTSXTypeScriptWebAssemblyYAMLXML  

This is critical for robot motion:

*   Maps rear wheel joints (joint\_rear\_left and joint\_rear\_right)
    
*   Subscribes to /cmd\_vel
    
*   Publishes /odom and TFs
    
*   Enables motion in simulation
    

### 📌 Important Design Choices

*   **Wheel joints** are continuous, suitable for rolling motion.
    
*   **LIDAR is fixed** to the base and raised above the body.
    
*   **Rear wheels drive** the robot using diff\_drive plugin.
    
*   **Frame names** match what the plugins expect (base, lidar, etc.)
    
*   **All links** have inertial, visual, and collision components to be physics-valid in Gazebo.
    

### ⚠️ Things to Check if Robot Doesn't Move

*   Are the wheel joint names in the plugin and correct?
    
*   Is the axis of rotation properly aligned (xyz="0 1 0" for Y-axis)?
    
*   Is cmd\_vel being published and matched correctly in the plugin with ?
    
*   Friction/surface parameters may be missing — can affect simulation movement.
    

🧱 Directory Tree Structure
---------------------------

Plain textANTLR4BashCC#CSSCoffeeScriptCMakeDartDjangoDockerEJSErlangGitGoGraphQLGroovyHTMLJavaJavaScriptJSONJSXKotlinLaTeXLessLuaMakefileMarkdownMATLABMarkupObjective-CPerlPHPPowerShell.propertiesProtocol BuffersPythonRRubySass (Sass)Sass (Scss)SchemeSQLShellSwiftSVGTSXTypeScriptWebAssemblyYAMLXML`   robot/  ├── src/  │   ├── bot_description/  │   │   ├── launch/  │   │   │   ├── view_model.launch.py  │   │   │   ├── spawn_robot.launch.py  │   │   │   └── gazebo_with_obstacle.launch.py  │   │   ├── urdf/  │   │   │   └── robot.xacro  │   │   └── setup.py  │   └── obstacle_stop/  │       ├── obstacle_stop_node.py  │       ├── setup.py  │       └── package.xml   `

⚠️ Current Status
-----------------

*   LIDAR is functional and detects objects.
    
*   TF tree is correct and robot is visualized.
    
*   /cmd\_vel commands are published.
    
*   **Robot is not moving yet in Gazebo.** Possibly due to joint or plugin issues (wheel control, friction, physics).
