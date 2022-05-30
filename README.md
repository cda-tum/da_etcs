# A Tool for the Generation, Verification and Optimization of ETCS Train Layouts/Schedules

Developers: Tom Peham and Robert Wille
## Overview

The European Train Control System (ETCS) strives to harmonize and improve train control systems throughout Europe. One particular aspect is the utilization of virtual subsections which allow for a much higher degree of freedom and provide significant potential for increasing the efficiency in today’s train schedules. However, exploiting this potential is a highly non-trivial task which, thus far, mainly relied on manual labor. In our work at the [Institute for Integrated Circuits](http://iic.jku.at/eda/) of the [Johannes Kepler University Linz](https://jku.at), we aim for introducing methods for design automation which aid designers of corresponding railway networks and train schedules.

In a first attempt (described in [[1]](https://iic.jku.at/files/eda/2021_date_automatic_design_verification_level3_etcs.pdf)), we developed an initial solution which utilizes satisfiability solvers to generate, verify, and optimize train layout and/or schedules. 

[[1]](https://iic.jku.at/files/eda/2021_date_automatic_design_verification_level3_etcs.pdf) Robert Wille and Tom Peham and Judith Przigoda and Nils Przigoda. **"Towards Automatic Design and Verification for Level 3 of the European Train Control System"**. Design, Automation and Test in Europe (DATE), 2021 ([pdf]( https://iic.jku.at/files/eda/2021_date_automatic_design_verification_level3_etcs.pdf))

If you have any questions, feel free to contact us via iic-etcs@jku.at or by creating an issue on GitHub.

## Usage

### System Requirements 

The package has been tested under Linux (Ubuntu 18.04, 64-bit) and should be compatible with any current version of g++ supporting C++17 and a minimum CMake version of 3.13.
The project also requires the Z3 theorem prover. It can be installed in any of the following ways:
- Under Ubuntu 18.04 and newer: `sudo apt install z3`
- Download pre-built binaries from https://github.com/Z3Prover/z3/releases and copy the files to the respective system directories
- Build Z3 from source and install it to the system

### Build and Run 

To build the tool go to the project folder and execute the following:

1) Configure CMake
    ```commandline
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release

2) Build the respective target.
    ```commandline
   cmake --build ./build --config Release
   ```

To use the tool run 

    ./da_etcs [option] <input file>

Options can be:
    
    -f ... verify given schedule, no layout generation
    -s ... verify schedule, layout generation
    -o ... optimize schedule, layout generation

If no options are given then all design tasks will be performed in that order.

#### Input File Format
 
The input file describes the track layout as an undirected graph and the train schedule. For now, however, you will find the current BNF of the file format given below. The example-folder contains some examples (in fact, the examples used in [[1]](https://iic.jku.at/files/eda/2021_date_automatic_design_verification_level3_etcs.pdf)). Moreover, the file runningExample_annotated.txt in the example-folder also includes an annotated version, which hopefully helps in getting an idea of the format.
```
#> input_file ::== [integer] [integer] nl nl (track nl)* nl [fork nl]* nl (train)*     (*number of nodes, max timesteps, ...*)
#> track ::== edge integer integer integer integer                                     (*edge, length, speed limit, ttd*)
#> fork ::== node node node
#> train ::== integer integer integer edge nl (stop nl)* nl                             (*speed, length, departure time, start edge, ...*)
#> node ::== 0 | 1 | ... | number_of_nodes-1
#> edge ::== node node
#> stop ::== integer edge                                                               (*arrival time, stop edge*)
#> nl ::== newline
```

#### Output Format
```
#> output_file ::== ("SAT" nl (vss nl)* nl (route nl)* ) | "UNSAT""
#> vss ::== node* (*nodes belonging to same vss*)
#> route ::== 0 ": " edge* nl | ... | maxTime-1 ": " edge* nl                                      (*train routes are ordered the same as trains in input file*)
```
## Reference

If you use our tool for your research, we would be thankful if you referred to it
by citing the following publication: 
````
@inproceedings{wille2021etcs,
   title={Towards Automatic Design and Verification for Level 3 of the European Train Control System},
   author={Robert Wille and Tom Peham and Judith Przigoda and Nils Przigoda},
   booktitle={Design, Automation and Test in Europe (DATE)},
   year={2021}
}
````
