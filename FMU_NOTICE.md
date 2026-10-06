# FMU provenance and scope

`fmus/MolonbotWheelPlant.fmu` was generated with the author's openSeRo modeling tool. The author explicitly authorized its publication in this repository under the repository's MIT License. The FMU archive contains `modelDescription.xml`, generated C sources, and Linux x86-64 and Windows x86-64 binaries.

The wheel model is didactic: a first-order angular-speed response with voltage saturation and integrated angle. It is **not** an identified model of any physical Molonbot motor or a safety component. The repository does not contain the private modeling-project ZIP, real robot sensor data, the Molonbot USD asset snapshot, personal correspondence, or third-party robot/room meshes.

The exact FMU SHA-256 is recorded in [the Demo A validation report](results/demo_A/validation/report.json) and [the four-wheel report](results/drive/report.json). Validate the FMU with `fmpy.validation.validate_fmu` before using it elsewhere.

Eight unchanged PNG screenshots of the author's SeRo_MBE/openSeRo modeling workflow are included in [the modeling gallery](docs/OPENSERO_MODELING.md). They were copied from the author's original `MolonbotWheelPlant.zip` archive (SHA-256 `6683d1be204afb84097febbf7cd9eea5011cf11ecc4eafca5e82890978686a05`). The archive's embedded `MolonbotWheelPlant.fmu` has SHA-256 `5cdb7566ef9dcffbca455a31fad3588fde8063e535002ec8899454965d695f76`, exactly matching the published FMU. The original ZIP, model project files, and other archive contents remain unpublished. See [image hashes](assets/opensero/screenshots/SHA256SUMS) to verify the copied screenshots.

The author explicitly approved publication of these eight screenshots under the repository's [MIT License](LICENSE). This permission covers the screenshots included here, not the unpublished ZIP, editable modeling projects, or the author's complete modeling application.
