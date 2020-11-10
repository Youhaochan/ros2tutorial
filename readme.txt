company_information
这个文件 用来练习操作一个自定义数据 
该自定义数据的类型是另外一个自定义数据

cpp_action
这个文件是用来 练习 动作meessage， 并且该server可以同时处理多个client  也就是用到了多线程

cpp_parameters
用来学习 如何在节点内部定义一个parameters 并且使用了launch来启动节点和修改参数

cpp_pubsub 
练习话题通讯， 内置了launch来启动节点

ros_toturials 
内部存放这tutlesim的源代码

tutorial_interfaces 
是一个用来存放自定义数据的功能包， 其他功能包可以使用本功能包自定义的数据

tutorials_client_server
是一个练习服务器客户端的代码

compisition_talker_listener
该功能包实现的功能也一个话题通讯，但是使用了 class executors 来将俩个节点放到一个线程中运行
除此之外，功能包里 有.hpp .cpp文件， 在Cmakelist中， 将其中俩个.cpp文件生成了库文件并存在bin当中，
使得另外一个.cpp文件可以添加这个库

开发一个工程的文件存放规则
将一些类的声明放在 include/B.hpp文件中，将对应的B.cpp文件放在src/文件下 
然后额外建立一个A.cpp文件 放在src/下。
在Cmakelist.txt 中 将B 类文件生成一个library， 供A类文件调用

所以一个大型的project 应该是 
create a workspace
create many packages inside the workspace. For example, we have one package A
inside the package A
we declare many class or function in the directory include/${PROJECT_NAME}/ and put the 
corresponding .cpp files in the src/
configure the CmakeList.txt file to generate the library so that a new .cpp file can include them


可能还有另外一种情况
一个功能包使用另外一个功能包生成的库文件
所以 可以这样 
在一个工作空间中 生成很多的功能包， 有些功能包不会生成对应的节点， 而是生成一个库文件， 可以给另外的功能包调用




