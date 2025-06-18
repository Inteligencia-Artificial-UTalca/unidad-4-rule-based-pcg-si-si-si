#include <iostream>
#include <vector>
#include <random>   // For random number generation
#include <chrono>   // For seeding the random number generator

// Define Map as a vector of vectors of integers.
// You can change 'int' to whatever type best represents your cells (e.g., char, bool).
using Map = std::vector<std::vector<int>>;

/**
 * @brief Prints the map (matrix) to the console.
 * @param map The map to print.
 */
void printMap(const Map& map) {
    std::cout << "--- Current Map ---" << std::endl;
    for (const auto& row : map) {
        for (int cell : row) {
            // Adapt this to represent your cells meaningfully (e.g., ' ' for empty, '#' for occupied).
            std::cout << cell << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "-------------------" << std::endl;
}

/**
 * @brief Function to implement the Cellular Automata logic.
 * It should take a map and return the updated map after one iteration.
 * @param currentMap The map in its current state.
 * @param W Width of the map.
 * @param H Height of the map.
 * @param R Radius of the neighbor window (e.g., 1 for 3x3, 2 for 5x5).
 * @param U Threshold to decide if the current cell becomes 1 or 0.
 * @return The map after applying the cellular automata rules.
 */
Map cellularAutomata(const Map& currentMap, int W, int H, int R, double U) {
    Map newMap = currentMap; // Initially, the new map is a copy of the current one

    // Iterate over each cell in the map
    for (int x = 0; x < H; ++x) {
        for (int y = 0; y < W; ++y) {
            int neighborCount = 0;
            int totalCells = 0;
            
            // Check all neighbors within radius R (square neighborhood)
            for (int dx = -R; dx <= R; ++dx) {
                for (int dy = -R; dy <= R; ++dy) {
                    int nx = x + dx;
                    int ny = y + dy;
                    
                    // Check if neighbor is within bounds
                    if (nx >= 0 && nx < H && ny >= 0 && ny < W) {
                        // Neighbor is within the map
                        if (currentMap[nx][ny] == 1) {
                            neighborCount++;
                        }
                        totalCells++;
                    } else {
                        // Neighbor is outside the map - treat as 1 (wall)
                        // This helps create natural borders
                        neighborCount++;
                        totalCells++;
                    }
                }
            }
            
            // Calculate the ratio of neighbors with value 1
            double ratio = static_cast<double>(neighborCount) / totalCells;
            
            // Apply the threshold rule
            if (ratio >= U) {
                newMap[x][y] = 1;
            } else {
                newMap[x][y] = 0;
            }
        }
    }

    return newMap;
}

/**
 * @brief Function to initialize the map with random values (0 or 1)
 * @param map The map to initialize
 * @param W Width of the map
 * @param H Height of the map
 * @param probability Probability of a cell being 1 (default 0.45)
 */
void initializeRandomMap(Map& map, int W, int H, double probability = 0.45) {
    // Random number generator
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine rng(seed);
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    
    for (int x = 0; x < H; ++x) {
        for (int y = 0; y < W; ++y) {
            if (dist(rng) < probability) {
                map[x][y] = 1;
            } else {
                map[x][y] = 0;
            }
        }
    }
}

/**
 * @brief Function to implement the Drunk Agent logic.
 * It should take a map and parameters controlling the agent's behavior,
 * then return the updated map after the agent performs its actions.
 *
 * @param currentMap The map in its current state.
 * @param W Width of the map.
 * @param H Height of the map.
 * @param J The number of times the agent "walks" (initiates a path).
 * @param I The number of steps the agent takes per "walk".
 * @param roomSizeX Max width of rooms the agent can generate.
 * @param roomSizeY Max height of rooms the agent can generate.
 * @param probGenerateRoom Probability (0.0 to 1.0) of generating a room at each step.
 * @param probIncreaseRoom If no room is generated, this value increases probGenerateRoom.
 * @param probChangeDirection Probability (0.0 to 1.0) of changing direction at each step.
 * @param probIncreaseChange If direction is not changed, this value increases probChangeDirection.
 * @param agentX Current X position of the agent (updated by reference).
 * @param agentY Current Y position of the agent (updated by reference).
 * @return The map after the agent's movements and actions.
 */
Map drunkAgent(const Map& currentMap, int W, int H, int J, int I, int roomSizeX, int roomSizeY,
               double probGenerateRoom, double probIncreaseRoom,
               double probChangeDirection, double probIncreaseChange,
               int& agentX, int& agentY) {
    Map newMap = currentMap;

    // Generador aleatorio
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine rng(seed);
    std::uniform_real_distribution<double> prob(0.0, 1.0);
    std::uniform_int_distribution<int> dirDist(0, 3);
    std::uniform_int_distribution<int> roomW(1, roomSizeX);
    std::uniform_int_distribution<int> roomH(1, roomSizeY);

    // Direcciones: izquierda, derecha, arriba, abajo
    std::vector<std::pair<int, int>> directions = {
        {0, -1}, {0, 1}, {-1, 0}, {1, 0}
    };

    // Dirección inicial aleatoria
    auto dir = directions[dirDist(rng)];
    int dx = dir.first;
    int dy = dir.second;

    for (int j = 0; j < J; ++j) {
        for (int i = 0; i < I; ++i) {
            // Marca el camino
            if (agentX >= 0 && agentX < H && agentY >= 0 && agentY < W) {
                newMap[agentX][agentY] = 1;
            }

            // Probabilidad de generar habitación centrada en el agente
            if (prob(rng) < probGenerateRoom) {
                int rw = roomW(rng);
                int rh = roomH(rng);
                int startX = agentX - rh / 2;
                int startY = agentY - rw / 2;

                for (int x = 0; x < rh; ++x) {
                    for (int y = 0; y < rw; ++y) {
                        int rx = startX + x;
                        int ry = startY + y;
                        if (rx >= 0 && rx < H && ry >= 0 && ry < W)
                            newMap[rx][ry] = 1;
                    }
                }

                probGenerateRoom = 0.1; // Reinicia
            } else {
                probGenerateRoom += probIncreaseRoom;
            }

            // Probabilidad de cambiar dirección
            if (prob(rng) < probChangeDirection) {
                auto newDir = directions[dirDist(rng)];
                dx = newDir.first;
                dy = newDir.second;
                probChangeDirection = 0.2; // Reinicia
            } else {
                probChangeDirection += probIncreaseChange;
            }

            // Mover al agente
            int newX = agentX + dx;
            int newY = agentY + dy;

            if (newX >= 0 && newX < H && newY >= 0 && newY < W) {
                agentX = newX;
                agentY = newY;
            } else {
                // Si se sale del mapa, cambia dirección
                auto newDir = directions[dirDist(rng)];
                dx = newDir.first;
                dy = newDir.second;
            }
        }
    }

    return newMap;
}

void runCellularAutomataOnly() {
    std::cout << "\n=== CELLULAR AUTOMATA SIMULATION ===" << std::endl;
    
    // --- Map Configuration ---
    int mapRows = 15;
    int mapCols = 25;
    Map myMap(mapRows, std::vector<int>(mapCols, 0));
    
    // Initialize with random noise
    initializeRandomMap(myMap, mapCols, mapRows, 0.45);
    
    std::cout << "\nInitial map state (random noise):" << std::endl;
    printMap(myMap);
    
    // Parameters
    int ca_W = mapCols;
    int ca_H = mapRows;
    int ca_R = 1;      // Radius
    double ca_U = 0.5; // Threshold
    int iterations = 5; // More iterations for better results
    
    // Apply cellular automata multiple times
    for (int i = 0; i < iterations; ++i) {
        std::cout << "\n--- Cellular Automata Iteration " << i + 1 << " ---" << std::endl;
        myMap = cellularAutomata(myMap, ca_W, ca_H, ca_R, ca_U);
        printMap(myMap);
    }
    
    std::cout << "\n--- Cellular Automata Simulation Finished ---" << std::endl;
}

void runDrunkAgentOnly() {
    std::cout << "\n=== DRUNK AGENT SIMULATION ===" << std::endl;
    
    // --- Map Configuration ---
    int mapRows = 15;
    int mapCols = 25;
    Map myMap(mapRows, std::vector<int>(mapCols, 0)); // Start with empty map
    
    // Agent's initial position
    std::default_random_engine rng(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_int_distribution<int> startX(0, mapRows - 1);
    std::uniform_int_distribution<int> startY(0, mapCols - 1);
    int drunkAgentX = startX(rng);
    int drunkAgentY = startY(rng);
    
    std::cout << "\nInitial empty map:" << std::endl;
    std::cout << "Agent starting position: (" << drunkAgentX << ", " << drunkAgentY << ")" << std::endl;
    printMap(myMap);
    
    // Parameters
    int da_W = mapCols;
    int da_H = mapRows;
    int da_J = 5;      // More walks
    int da_I = 12;     // More steps per walk
    int da_roomSizeX = 5;
    int da_roomSizeY = 4;
    double da_probGenerateRoom = 0.15;
    double da_probIncreaseRoom = 0.08;
    double da_probChangeDirection = 0.25;
    double da_probIncreaseChange = 0.05;
    
    int iterations = 3;
    
    // Apply drunk agent multiple times
    for (int i = 0; i < iterations; ++i) {
        std::cout << "\n--- Drunk Agent Iteration " << i + 1 << " ---" << std::endl;
        myMap = drunkAgent(myMap, da_W, da_H, da_J, da_I, da_roomSizeX, da_roomSizeY,
                          da_probGenerateRoom, da_probIncreaseRoom,
                          da_probChangeDirection, da_probIncreaseChange,
                          drunkAgentX, drunkAgentY);
        std::cout << "Agent position: (" << drunkAgentX << ", " << drunkAgentY << ")" << std::endl;
        printMap(myMap);
    }
    
    std::cout << "\n--- Drunk Agent Simulation Finished ---" << std::endl;
}

void runCombinedSimulation() {
    std::cout << "\n=== COMBINED SIMULATION ===" << std::endl;
    
    // --- Map Configuration ---
    int mapRows = 15;
    int mapCols = 25;
    Map myMap(mapRows, std::vector<int>(mapCols, 0));
    
    // Initialize with random noise for cellular automata
    initializeRandomMap(myMap, mapCols, mapRows, 0.45);
    
    // Agent's initial position
    std::default_random_engine rng(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_int_distribution<int> startX(0, mapRows - 1);
    std::uniform_int_distribution<int> startY(0, mapCols - 1);
    int drunkAgentX = startX(rng);
    int drunkAgentY = startY(rng);
    
    std::cout << "\nInitial map state (random noise):" << std::endl;
    printMap(myMap);
    
    // Parameters
    int numIterations = 3;
    
    // Cellular Automata Parameters
    int ca_W = mapCols;
    int ca_H = mapRows;
    int ca_R = 1;
    double ca_U = 0.5;
    
    // Drunk Agent Parameters
    int da_W = mapCols;
    int da_H = mapRows;
    int da_J = 3;
    int da_I = 8;
    int da_roomSizeX = 4;
    int da_roomSizeY = 3;
    double da_probGenerateRoom = 0.1;
    double da_probIncreaseRoom = 0.05;
    double da_probChangeDirection = 0.2;
    double da_probIncreaseChange = 0.03;
    
    // Main simulation loop
    for (int iteration = 0; iteration < numIterations; ++iteration) {
        std::cout << "\n--- Combined Iteration " << iteration + 1 << " ---" << std::endl;
        
        // Apply cellular automata
        std::cout << "Applying Cellular Automata..." << std::endl;
        myMap = cellularAutomata(myMap, ca_W, ca_H, ca_R, ca_U);
        std::cout << "After Cellular Automata:" << std::endl;
        printMap(myMap);
        
        // Apply drunk agent
        std::cout << "Applying Drunk Agent..." << std::endl;
        myMap = drunkAgent(myMap, da_W, da_H, da_J, da_I, da_roomSizeX, da_roomSizeY,
                          da_probGenerateRoom, da_probIncreaseRoom,
                          da_probChangeDirection, da_probIncreaseChange,
                          drunkAgentX, drunkAgentY);
        std::cout << "After Drunk Agent:" << std::endl;
        printMap(myMap);
    }
    
    // Final smoothing
    std::cout << "\n--- Final Smoothing ---" << std::endl;
    for (int i = 0; i < 2; ++i) {
        myMap = cellularAutomata(myMap, ca_W, ca_H, ca_R, ca_U);
    }
    
    std::cout << "Final combined map:" << std::endl;
    printMap(myMap);
    
    std::cout << "\n--- Combined Simulation Finished ---" << std::endl;
}

int main() {
    std::cout << "=== MAP GENERATION SIMULATION ===" << std::endl;
    std::cout << "Choose simulation type:" << std::endl;
    std::cout << "1. Cellular Automata Only" << std::endl;
    std::cout << "2. Drunk Agent Only" << std::endl;
    std::cout << "3. Combined Simulation" << std::endl;
    std::cout << "4. Exit" << std::endl;
    std::cout << "Enter your choice (1-4): ";
    
    int choice;
    std::cin >> choice;
    
    switch (choice) {
        case 1:
            runCellularAutomataOnly();
            break;
        case 2:
            runDrunkAgentOnly();
            break;
        case 3:
            runCombinedSimulation();
            break;
        case 4:
            std::cout << "Exiting program..." << std::endl;
            return 0;
        default:
            std::cout << "Invalid choice! Running combined simulation by default." << std::endl;
            runCombinedSimulation();
            break;
    }
    
    std::cout << "\nLegend: 0 = Empty space, 1 = Wall/Filled" << std::endl;
    std::cout << "Program finished." << std::endl;
    
    return 0;
}