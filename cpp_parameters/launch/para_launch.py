from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
	        Node(
	            package="cpp_parameters",
	            namespace='parameters_one',
	            executable="parameter_node",
	            #here we replace the name of node with custom_parameter_node
	            name="custom_parameter_node", 
	            #By adding the two lines below, we ensure our output is printed in our console.
	            output="screen",
	            emulate_tty=True,
	            parameters=[
	                {"my_parameter": "earth"}
	            ]
	        ),
	        Node(
				package="cpp_parameters",
	            executable="parameter_node",
	            #here we replace the name of node with custom_parameter_node
	            name="custom_parameter_node2", 
	            #By adding the two lines below, we ensure our output is printed in our console.
	            output="screen",
	            emulate_tty=True,
	            parameters=[
	                {"my_parameter": "haochan.you"}
	            ]
	        	)
	    
	    ])