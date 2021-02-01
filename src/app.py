# imports
from __future__ import print_function
from geopy.geocoders import Nominatim
from geopy.distance import geodesic
import numpy as np
from pprint import pprint

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

test_places = ['Oraiokastro','Kalamaria','Pefka', 'Thermi', 'Chalkidiki', 'Panorama, Thessaloniki']
places = {}


def init():
    preset = input('Use preset places? [y/n] ')
    if preset != 'y':
        input_places = []
        depot = input ('Enter your depot: \n')
        input_places.append(depot)
        setting_up = True
        while setting_up:
            place = input('Name of the place to be added: ([x] to exit) \n')
            if place != 'x':
                input_places.append(place)
            else:
                setting_up = False
        return input_places
    else:
        return test_places


def create_data_model(place_data,vehicleNum):
    geolocator = Nominatim(user_agent="app")
    for place in place_data:
        location = geolocator.geocode(place)
        places[place] = (location.latitude, location.longitude)
        print(location)

    distance_matrix = np.ones([len(places), len(places)])
    for i in range(len(place_data)):
        for j in range(i, len(place_data)):
            if i != j:
                dist = geodesic(places[place_data[i]], places[place_data[j]]).kilometers
                distance_matrix[i][j] = dist
                distance_matrix[j][i] = dist
            else:
                distance_matrix[i][i] = 0

    data = {'distance_matrix': [distance_matrix[i].tolist() for i in range(len(place_data))],
            'num_vehicles': vehicleNum, 'depot': 0, 'place_names': place_data}
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    max_route_distance = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        plan_output_names = ''
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += ' {} -> '.format(manager.IndexToNode(index))
            plan_output_names += ' {} -> '.format(data['place_names'][index])
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += '{}\n'.format(manager.IndexToNode(index))
        plan_output_names += '{}\n'.format(data['place_names'][0])
        plan_output_names += 'Distance of the route: {} km\n'.format(route_distance)
        # print(plan_output) # uncomment this to display the route by indices
        print(plan_output_names)
        max_route_distance = max(route_distance, max_route_distance)
    print('Maximum of the route distances: {}km'.format(max_route_distance))


def main():

    places_choice = init()

    data = create_data_model(places_choice,1)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        3000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)


if __name__ == "__main__":
    main()

