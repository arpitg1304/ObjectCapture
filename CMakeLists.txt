cmake_minimum_required(VERSION 3.9)
project(ObjectCapture)

set(CMAKE_CXX_STANDARD 11)

add_executable(ObjectCapture
        detect_blue.py
        fk_ik.py
        getData.py
        object_capture.py
        README.md
        scene1.ttt)
