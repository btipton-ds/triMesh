# triMesh Library

TriMesh is a simple triangular mesh package. It was built as a foundation for [VulkanQuickStart](https://github.com/btipton-ds/VulkanQuickStart) and other applications under development at Dark Sky Innovative solutions. It is a simpler and cheaper altrnative to other triangular mesh libraries.

## Getting Started

The project was developed under MS Visual Studio 2019 as a Make File Project. It was also built with gcc under WSL. That environment will work best.

### Prerequisites

The Eigen matrix algebra package may be substituted for the project's vector3.h, but is not required.

### Installing

1 Clone the repository  
2 Launch VS 2019 (2017 shuould work also) - get the community edition if you don't have a copy  
3 Skip the entry screen using the 'without code' option  
4 Open the project  
5 Choose makefile project and select the root CMakeLists.txt  

Build the libraries you need.

## Running the tests

Compile the enerMeshTest target and run it.

I am an advocate of minimal low level testing and extensive automated workflow testing. Such tests don't exist at the time of this writing.

## Deployment

It may be linked in as a static library. No shared library builds are provided.

## Built With

* Built with Visual Studio 2019 (https://visualstudio.microsoft.com/downloads/)

## Contributing

Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.


## Authors

**Robert R (Bob) Tipton** - (btipton <-> darkskyinnovation.com)

## License

This project is licensed under the GPLv3 License - see <https://www.gnu.org/licenses/> file for details

