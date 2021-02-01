## Project structure - TSP

### Description

Simple Python app solving the Vehicle Routing problem. developed for a friendly company

### Data given:

- Places to be visited
- Depot
- Number of Vehicles

### Returns:

- Route that has to be followed by each vehicle
- Measures of the distance

### Tools:

- https://developers.googley.com/optimization/routing/tsp - TSP algorithm
- Libs: geopy, numpy, OR tools

### Limitations:

- Using Geodesic distance instead of using the actual roads
  (could use gmaps API in the future)
