from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([

		Node(
			    package="cpp_pubsub",
			    # 这个命名空间也会改变话题名称，所以订阅者也要在相同命名空间下才可以互相通信
	            #namespace='pub',
	            executable="talker",
	            #here we replace the name of node with custom_parameter_node
	            name="talker_node", 
	            # 用来重映射话题的名字
	            remappings=[
           		 ("talker_listener", "pubsub")
       			 ],
	            #By adding the two lines below, we ensure our output is printed in our console.
	            output="screen",
	            emulate_tty=True
	         
			)
		,
		Node(
				package = "cpp_pubsub",
				#namespace = "sub",
				executable = "listener",
				name = "listerner_node",
				 remappings=[
           		 ("talker_listener", "pubsub")
       			 ],
			)


		]
		)

#another way to write the launch file
# def generate_launch_description():
#     ld = LaunchDescription()
#     talker_node = Node(
#         package="demo_nodes_cpp",
#         executable="talker",
#     )
#     listener_node = Node(
#         package="demo_nodes_py",
#         executable="listener"
#     )
#     ld.add_action(talker_node)
#     ld.add_action(listener_node)
#     return ld