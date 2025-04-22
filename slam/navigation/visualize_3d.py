import argparse
import pyvista as pv  # type: ignore


def main():
    # Define the command line arguments using argparse
    parser = argparse.ArgumentParser(description="Visualize 3D files.")
    parser.add_argument(
        "-i", "--input", required=True, nargs="+", help="Paths to the input files."
    )

    # Parse the arguments
    args = parser.parse_args()

    # Create a plotter object
    plotter = pv.Plotter()

    # Load and add each mesh to the plotter
    for input_file in args.input:
        mesh = pv.read(input_file)
        plotter.add_mesh(mesh)

    # Display the plot
    plotter.show()


if __name__ == "__main__":
    main()
