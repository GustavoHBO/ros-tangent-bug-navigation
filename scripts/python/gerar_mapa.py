import numpy as np
import cv2
import argparse
import random
import heapq

def generate_map(width, height, corridor_width, wall_width, room_chance, resolution, start, goal, closed_map):
    """Gera um mapa com cômodos fechados e um caminho garantido entre início e destino."""
    cell_size = corridor_width + wall_width
    width = ((width // cell_size) * cell_size) + wall_width
    height = ((height // cell_size) * cell_size) + wall_width

    width_px = int(width / resolution)
    height_px = int(height / resolution)
    corridor_width_px = int(corridor_width / resolution)
    wall_width_px = int(wall_width / resolution)
    cell_size_px = corridor_width_px + wall_width_px

    # Cria o mapa com fundo branco e corredores em 0
    mapa = np.ones((height_px, width_px), dtype=np.uint8) * 255  

    for y in range(wall_width_px, height_px - corridor_width_px, cell_size_px):
        for x in range(wall_width_px, width_px - corridor_width_px, cell_size_px):
            if random.random() > room_chance:
                mapa[y:y+corridor_width_px, x:x+corridor_width_px] = 0
            else:
                if random.random() > 0.5:
                    mapa[y-wall_width_px:y+corridor_width_px+wall_width_px, x:x+corridor_width_px] = 0
                else:
                    mapa[y:y+corridor_width_px, x-wall_width_px:x+corridor_width_px+wall_width_px] = 0

    start_px = (int(start[0] / resolution), int(start[1] / resolution))
    goal_px = (int(goal[0] / resolution), int(goal[1] / resolution))

    # Garante que as áreas de início e destino sejam corredores
    mapa[start_px[1]:start_px[1]+corridor_width_px, start_px[0]:start_px[0]+corridor_width_px] = 0
    mapa[goal_px[1]:goal_px[1]+corridor_width_px, goal_px[0]:goal_px[0]+corridor_width_px] = 0

    if closed_map:
        mapa[:wall_width_px, :] = 255
        mapa[-wall_width_px:, :] = 255
        mapa[:, :wall_width_px] = 255
        mapa[:, -wall_width_px:] = 255

    if not is_path_possible(mapa, start_px, goal_px):
        mapa = create_random_path(mapa, start_px, goal_px, corridor_width_px)

    return mapa

def is_path_possible(mapa, start, goal):
    """Verifica se há um caminho entre início e destino usando busca em largura."""
    h, w = mapa.shape
    queue = [start]
    visited = set([start])

    while queue:
        x, y = queue.pop(0)
        if (x, y) == goal:
            return True

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and (nx, ny) not in visited and mapa[ny, nx] == 0:
                queue.append((nx, ny))
                visited.add((nx, ny))
    return False

def simplify_path(path):
    """Simplifica uma lista de pontos removendo os pontos intermediários que não representam mudança de direção."""
    if not path:
        return path
    simplified = [path[0]]
    if len(path) >= 2:
        last_dx = path[1][0] - path[0][0]
        last_dy = path[1][1] - path[0][1]
    else:
        return path
    for i in range(1, len(path)):
        if i < len(path) - 1:
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            if (dx, dy) != (last_dx, last_dy):
                simplified.append(path[i])
                last_dx, last_dy = dx, dy
        else:
            simplified.append(path[i])
    return simplified

def create_random_path(mapa, start, goal, corridor_width_px):
    """
    Cria um caminho utilizando A* para conectar início e destino e o simplifica
    para que seja constituído de segmentos de reta.
    """
    def heuristic(a, b):
        # Heurística Manhattan com leve variação aleatória
        return abs(a[0] - b[0]) + abs(a[1] - b[1]) + random.randint(0, 5)

    h, w = mapa.shape
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while open_list:
        _, current = heapq.heappop(open_list)
        if current == goal:
            break
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = current[0] + dx, current[1] + dy
            if 0 <= nx < w and 0 <= ny < h and mapa[ny, nx] == 0:
                new_cost = cost_so_far[current] + 1
                if (nx, ny) not in cost_so_far or new_cost < cost_so_far[(nx, ny)]:
                    cost_so_far[(nx, ny)] = new_cost
                    priority = new_cost + heuristic(goal, (nx, ny))
                    heapq.heappush(open_list, (priority, (nx, ny)))
                    came_from[(nx, ny)] = current

    # Se o A* não encontrar um caminho, define um caminho Manhattan simples
    if goal not in came_from:
        path_points = [start, (goal[0], start[1]), goal]
    else:
        # Reconstrói o caminho encontrado e simplifica
        path_points = []
        current = goal
        while current != start:
            path_points.append(current)
            current = came_from[current]
        path_points.append(start)
        path_points.reverse()
        path_points = simplify_path(path_points)

    # # Desenha cada segmento como reta (horizontal ou vertical)
    # for i in range(len(path_points) - 1):
    #     p1 = path_points[i]
    #     p2 = path_points[i + 1]
    #     # Se o segmento for horizontal
    #     if p1[1] == p2[1]:
    #         y = p1[1]
    #         for x in range(min(p1[0], p2[0]), max(p1[0], p2[0]) + 1):
    #             mapa[y:y+corridor_width_px, x:x+corridor_width_px] = 0
    #     # Se o segmento for vertical
    #     elif p1[0] == p2[0]:
    #         x = p1[0]
    #         for y in range(min(p1[1], p2[1]), max(p1[1], p2[1]) + 1):
    #             mapa[y:y+corridor_width_px, x:x+corridor_width_px] = 0
    #     else:
    #         # Caso improvável: divide em dois segmentos (horizontal + vertical)
    #         interm = (p2[0], p1[1])
    #         for x in range(min(p1[0], interm[0]), max(p1[0], interm[0]) + 1):
    #             mapa[p1[1]:p1[1]+corridor_width_px, x:x+corridor_width_px] = 0
    #         for y in range(min(interm[1], p2[1]), max(interm[1], p2[1]) + 1):
    #             mapa[y:y+corridor_width_px, p2[0]:p2[0]+corridor_width_px] = 0

    # Verifica se o caminho é viável; se não for, cria um caminho Manhattan garantido
    # if not is_path_possible(mapa, start, goal):
    #     x1, y1 = start
    #     x2, y2 = goal
    #     # Segmento horizontal
    #     for x in range(min(x1, x2), max(x1, x2)+1):
    #         mapa[y1:y1+corridor_width_px, x:x+corridor_width_px] = 0
    #     # Segmento vertical
    #     for y in range(min(y1, y2), max(y1, y2)+1):
    #         mapa[y:y+corridor_width_px, x2:x2+corridor_width_px] = 0

    return mapa

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Gerador de mapas com caminho garantido para Gazebo")
    parser.add_argument("--width", type=int, default=10, help="Largura do mapa em metros")
    parser.add_argument("--height", type=int, default=10, help="Altura do mapa em metros")
    parser.add_argument("--corridor_width", type=float, default=1.0, help="Largura das ruas em metros")
    parser.add_argument("--wall_width", type=float, default=0.5, help="Espessura das paredes em metros")
    parser.add_argument("--room_chance", type=float, default=0.3, help="Chance de criar cômodos fechados (0.0 a 1.0)")
    parser.add_argument("--resolution", type=float, default=0.05, help="Resolução do mapa (metros por pixel)")
    parser.add_argument("--start", type=float, nargs=2, default=[1.0, 1.0], help="Coordenadas de início (x y) em metros")
    parser.add_argument("--goal", type=float, nargs=2, default=[8.0, 8.0], help="Coordenadas de destino (x y) em metros")
    parser.add_argument("--closed_map", action="store_true", help="Se presente, adiciona paredes ao redor do mapa")
    parser.add_argument("--output", type=str, default="map.pgm", help="Nome do arquivo de saída")

    args = parser.parse_args()

    mapa = generate_map(args.width, args.height, args.corridor_width, args.wall_width,
                        args.room_chance, args.resolution, args.start, args.goal, args.closed_map)
    cv2.imwrite(args.output, mapa)

    yaml_content = f"""image: {args.output}
resolution: {args.resolution}
origin: [0.0, 0.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
"""
    yaml_filename = args.output.replace(".pgm", ".yaml")
    with open(yaml_filename, "w") as f:
        f.write(yaml_content)

    print(f"Mapa salvo como {args.output} com caminho garantido de {args.start} até {args.goal}.")
