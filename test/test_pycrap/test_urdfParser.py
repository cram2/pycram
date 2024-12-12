from pycrap.urdf_parser import URDFParser

def main():
    # Path to the URDF file
    urdf_file = "/pycram/resources/objects/kitchen-small.urdf"  # Replace with the actual URDF file path

    # Output directory for generated files
    output_dir = "/home/sorin/playground/parsedURDF"  # Replace with the desired output directory path

    # Initialize the URDFParser
    parser = URDFParser(urdf_file, output_dir)

    # Generate the `classes.py` file
    parser.generate_all()

if __name__ == "__main__":
    main()
