import sys
import xml.etree.ElementTree as ET

def sort_coordinates(coordinates):
    """Sort coordinates from left to right, and for shapes, top to bottom, left to right."""
    return sorted(coordinates, key=lambda point: (point[1], point[0]))

def interpolate_points(start, end):
    """Generate intermediate points from start to end."""
    x1, y1 = start
    x2, y2 = end
    points = []

    # Calculate step increments
    dx = 1 if x2 > x1 else -1 if x2 < x1 else 0
    dy = 1 if y2 > y1 else -1 if y2 < y1 else 0

    # Move along the x-axis or y-axis
    x, y = x1, y1
    points.append((x, y))
    while (x, y) != (x2, y2):
        if x != x2:
            x += dx
        if y != y2:
            y += dy
        points.append((x, y))
    return points

def extract_coordinates(element):
    """Extract coordinates from specific SVG elements."""
    tag = element.tag.lower()
    if tag.endswith('line'):
        x1 = element.attrib.get('x1', '0')
        y1 = element.attrib.get('y1', '0')
        x2 = element.attrib.get('x2', '0')
        y2 = element.attrib.get('y2', '0')
        coordinates = [(float(x1), float(y1)), (float(x2), float(y2))]
        return sorted(coordinates, key=lambda point: point[0])  # Sort left to right
    elif tag.endswith('path'):
        d = element.attrib.get('d', '')
        # Note: For simplicity, this assumes `d` contains only "M" and "L" commands.
        coordinates = []
        for command in d.split():
            if command[0].upper() in ('M', 'L'):
                coords = command[1:].split(',')
                coordinates.append((float(coords[0]), float(coords[1])))
        return sort_coordinates(coordinates)
    elif tag.endswith('rect'):
        x = float(element.attrib.get('x', '0'))
        y = float(element.attrib.get('y', '0'))
        width = float(element.attrib.get('width', '0'))
        height = float(element.attrib.get('height', '0'))
        # Rectangle corners: top-left, top-right, bottom-right, bottom-left
        coordinates = [
            (x, y),
            (x + width, y),
            (x + width, y + height),
            (x, y + height)
        ]
        return sort_coordinates(coordinates)
    return []

def get_full_path_coordinates(svg_file):
    """Extract and interpolate all coordinates from black lines and rectangles in the SVG file."""
    try:
        tree = ET.parse(svg_file)
        root = tree.getroot()
        all_paths = []  # 2D list to store each path's points

        for element in root.iter():
            style = element.attrib.get('style', '')
            stroke = element.attrib.get('stroke', '')

            # Check if the element is black (by stroke or style)
            if 'black' in style.lower() or stroke.lower() == 'black':
                coordinates = extract_coordinates(element)
                if coordinates:
                    path_points = []
                    for i in range(len(coordinates) - 1):
                        start = coordinates[i]
                        end = coordinates[i + 1]
                        path_points.extend(interpolate_points(start, end))
                    all_paths.append(path_points)

        return all_paths
    except ET.ParseError as e:
        print(f"Error parsing SVG file: {e}")
        return []
    except Exception as e:
        print(f"An error occurred: {e}")
        return []

def main():
    print("SVG Line Tracker")

    # Automatically use example.svg as the input file
    svg_file = "example.svg"

    print("Processing SVG file...")
    all_paths = get_full_path_coordinates(svg_file)

    for path_index, path_points in enumerate(all_paths):
        print(f"Path {path_index + 1}:")
        for point in path_points:
            print(f"Coordinate: x={point[0]}, y={point[1]}")

    # Print only the first path
    if all_paths:
        print("\nFirst path coordinates:")
        for point in all_paths[0]:
            print(f"Coordinate: x={point[0]}, y={point[1]}")

if __name__ == "__main__":
    main()
