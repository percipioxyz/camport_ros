cmake_minimum_required(VERSION 2.8.3)
project(percipio_launch)

find_package(catkin)

catkin_package()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

install(FILES parameters.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})