# FMU provenance and scope

`fmus/MolonbotWheelPlant.fmu` was generated with the author's openSeRo modeling tool. The author explicitly authorized its publication in this repository under the repository's MIT License. The FMU archive contains `modelDescription.xml`, generated C sources, and Linux x86-64 and Windows x86-64 binaries.

The wheel model is didactic: a first-order angular-speed response with voltage saturation and integrated angle. It is **not** an identified model of any physical Molonbot motor or a safety component. The repository does not contain the private modeling-project ZIP, real robot sensor data, the Molonbot USD asset snapshot, personal correspondence, or third-party robot/room meshes.

The exact FMU SHA-256 is recorded in [results/demo_a/report.json](results/demo_a/report.json) and [results/drive/report.json](results/drive/report.json). Validate the FMU with `fmpy.validation.validate_fmu` before using it elsewhere.
