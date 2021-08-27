#include "Pathfinding.h"

using namespace std;

vector <Point> pathfinding_Astar(double lat_drone, double lon_drone, double h_drone, double lat_dest, double lon_dest, double h_dest, int q_obs){
    //Fazer o construtor e destrutor copia da malha pode ser uma otima ideia

    //Iicializacao dos valores
    Malha M = transf(lat_drone, lon_drone, h_drone);
    Point destiny(lat_dest, lon_dest, h_dest);
    int arrived = 0; //colocar bool
    int pos = -1;
    int pos_aux;
    double dist_pts;
    double distance;
    double distance_aux;
    vector <Point> points_list;
    //vector <Point> dijkstra_list;

    //Colocando obstaculo aleatorio
    //gen_rand_obs(M, q_obs);
    M.malha[14].set_obs_true();
    M.malha[14 + 169].set_obs_true();
    M.malha[14 + 169 + 169].set_obs_true();
    M.malha[14 + 3 * 169].set_obs_true();

    M.malha[17].set_obs_true();
    M.malha[17 + 169].set_obs_true();
    M.malha[17 + 169 + 169].set_obs_true();
    M.malha[17 + 3 * 169].set_obs_true();
    M.malha[17 + 4 * 169].set_obs_true();
    M.malha[17 + 5 * 169].set_obs_true();
    M.malha[17 + 6 * 169].set_obs_true();

    M.malha[24].set_obs_true();
    M.malha[24 + 169].set_obs_true();
    M.malha[24 + 169 + 169].set_obs_true();
    M.malha[24 + 3 * 169].set_obs_true();

    M.malha[30].set_obs_true();
    M.malha[30 + 169].set_obs_true();
    M.malha[30 + 169 + 169].set_obs_true();
    M.malha[30 + 3 * 169].set_obs_true();
    M.malha[30 + 4 * 169].set_obs_true();
    M.malha[30 + 5 * 169].set_obs_true();
    M.malha[30 + 6 * 169].set_obs_true();

    M.malha[43].set_obs_true();
    M.malha[43 + 169].set_obs_true();
    M.malha[43 + 169 + 169].set_obs_true();
    M.malha[43 + 3 * 169].set_obs_true();
    M.malha[43 + 4 * 169].set_obs_true();
    M.malha[43 + 5 * 169].set_obs_true();
    M.malha[43 + 6 * 169].set_obs_true();

    M.malha[44].set_obs_true();
    M.malha[44 + 169].set_obs_true();
    M.malha[44 + 169 + 169].set_obs_true();
    M.malha[44 + 3 * 169].set_obs_true();
    M.malha[44 + 4 * 169].set_obs_true();
    M.malha[44 + 5 * 169].set_obs_true();
    M.malha[44 + 6 * 169].set_obs_true();

    M.malha[45].set_obs_true();
    M.malha[45 + 169].set_obs_true();
    M.malha[45 + 169 + 169].set_obs_true();
    M.malha[45 + 3 * 169].set_obs_true();
    M.malha[45 + 4 * 169].set_obs_true();
    M.malha[45 + 5 * 169].set_obs_true();
    M.malha[45 + 6 * 169].set_obs_true();

    M.malha[46].set_obs_true();
    M.malha[46 + 169].set_obs_true();
    M.malha[46 + 169 + 169].set_obs_true();
    M.malha[46 + 3 * 169].set_obs_true();
    M.malha[46 + 4 * 169].set_obs_true();
    M.malha[46 + 5 * 169].set_obs_true();
    M.malha[46 + 6 * 169].set_obs_true();

    M.malha[47].set_obs_true();
    M.malha[47 + 169].set_obs_true();
    M.malha[47 + 169 + 169].set_obs_true();
    M.malha[47 + 3 * 169].set_obs_true();
    M.malha[47 + 4 * 169].set_obs_true();
    M.malha[47 + 5 * 169].set_obs_true();
    M.malha[47 + 6 * 169].set_obs_true();

    M.malha[48].set_obs_true();
    M.malha[48 + 169].set_obs_true();
    M.malha[48 + 169 + 169].set_obs_true();
    M.malha[48 + 3 * 169].set_obs_true();
    M.malha[48 + 4 * 169].set_obs_true();
    M.malha[48 + 5 * 169].set_obs_true();
    M.malha[48 + 6 * 169].set_obs_true();

    M.malha[56].set_obs_true();
    M.malha[56 + 169].set_obs_true();
    M.malha[56 + 169 + 169].set_obs_true();
    M.malha[56 + 3 * 169].set_obs_true();
    M.malha[56 + 4 * 169].set_obs_true();
    M.malha[56 + 5 * 169].set_obs_true();
    M.malha[56 + 6 * 169].set_obs_true();

    M.malha[61].set_obs_true();
    M.malha[61 + 169].set_obs_true();
    M.malha[61 + 169 + 169].set_obs_true();
    M.malha[61 + 3 * 169].set_obs_true();
    M.malha[61 + 4 * 169].set_obs_true();
    M.malha[61 + 5 * 169].set_obs_true();
    M.malha[61 + 6 * 169].set_obs_true();

    M.malha[69].set_obs_true();
    M.malha[69 + 169].set_obs_true();
    M.malha[69 + 169 + 169].set_obs_true();
    M.malha[69 + 3 * 169].set_obs_true();
    M.malha[69 + 4 * 169].set_obs_true();
    M.malha[69 + 5 * 169].set_obs_true();
    M.malha[69 + 6 * 169].set_obs_true();

    M.malha[72].set_obs_true();
    M.malha[72 + 169].set_obs_true();
    M.malha[72 + 169 + 169].set_obs_true();
    M.malha[72 + 3 * 169].set_obs_true();

    M.malha[74].set_obs_true();
    M.malha[74 + 169].set_obs_true();
    M.malha[74 + 169 + 169].set_obs_true();
    M.malha[74 + 3 * 169].set_obs_true();
    M.malha[74 + 4 * 169].set_obs_true();
    M.malha[74 + 5 * 169].set_obs_true();
    M.malha[74 + 6 * 169].set_obs_true();

    M.malha[87].set_obs_true();
    M.malha[87 + 169].set_obs_true();
    M.malha[87 + 169 + 169].set_obs_true();
    M.malha[87 + 3 * 169].set_obs_true();
    M.malha[87 + 4 * 169].set_obs_true();
    M.malha[87 + 5 * 169].set_obs_true();
    M.malha[87 + 6 * 169].set_obs_true();

    M.malha[93].set_obs_true();
    M.malha[93 + 169].set_obs_true();
    M.malha[93 + 169 + 169].set_obs_true();
    M.malha[93 + 3 * 169].set_obs_true();
    M.malha[93 + 4 * 169].set_obs_true();
    M.malha[93 + 5 * 169].set_obs_true();
    M.malha[93 + 6 * 169].set_obs_true();

    M.malha[118].set_obs_true();
    M.malha[118 + 169].set_obs_true();
    M.malha[118 + 169 + 169].set_obs_true();
    M.malha[118 + 3 * 169].set_obs_true();
    M.malha[118 + 4 * 169].set_obs_true();
    M.malha[118 + 5 * 169].set_obs_true();
    M.malha[118 + 6 * 169].set_obs_true();

    M.malha[119].set_obs_true();
    M.malha[119 + 169].set_obs_true();
    M.malha[119 + 169 + 169].set_obs_true();
    M.malha[119 + 3 * 169].set_obs_true();
    M.malha[119 + 4 * 169].set_obs_true();
    M.malha[119 + 5 * 169].set_obs_true();
    M.malha[119 + 6 * 169].set_obs_true();

    M.malha[120].set_obs_true();
    M.malha[120 + 169].set_obs_true();
    M.malha[120 + 169 + 169].set_obs_true();
    M.malha[120 + 3 * 169].set_obs_true();
    M.malha[120 + 4 * 169].set_obs_true();
    M.malha[120 + 5 * 169].set_obs_true();
    M.malha[120 + 6 * 169].set_obs_true();

    M.malha[121].set_obs_true();
    M.malha[121 + 169].set_obs_true();
    M.malha[121 + 169 + 169].set_obs_true();
    M.malha[121 + 3 * 169].set_obs_true();
    M.malha[121 + 4 * 169].set_obs_true();
    M.malha[121 + 5 * 169].set_obs_true();
    M.malha[121 + 6 * 169].set_obs_true();

    M.malha[124].set_obs_true();
    M.malha[124 + 169].set_obs_true();
    M.malha[124 + 169 + 169].set_obs_true();
    M.malha[124 + 3 * 169].set_obs_true();
    M.malha[124 + 4 * 169].set_obs_true();
    M.malha[124 + 5 * 169].set_obs_true();
    M.malha[124 + 6 * 169].set_obs_true();

    M.malha[125].set_obs_true();
    M.malha[125 + 169].set_obs_true();
    M.malha[125 + 169 + 169].set_obs_true();
    M.malha[125 + 3 * 169].set_obs_true();
    M.malha[125 + 4 * 169].set_obs_true();
    M.malha[125 + 5 * 169].set_obs_true();
    M.malha[125 + 6 * 169].set_obs_true();

    M.malha[126].set_obs_true();
    M.malha[126 + 169].set_obs_true();
    M.malha[126 + 169 + 169].set_obs_true();
    M.malha[126 + 3 * 169].set_obs_true();
    M.malha[126 + 4 * 169].set_obs_true();
    M.malha[126 + 5 * 169].set_obs_true();
    M.malha[126 + 6 * 169].set_obs_true();

    M.malha[140].set_obs_true();
    M.malha[140 + 169].set_obs_true();
    M.malha[140 + 169 + 169].set_obs_true();
    M.malha[140 + 3 * 169].set_obs_true();
    M.malha[140 + 4 * 169].set_obs_true();
    M.malha[140 + 5 * 169].set_obs_true();
    M.malha[140 + 6 * 169].set_obs_true();

    M.malha[141].set_obs_true();
    M.malha[141 + 169].set_obs_true();
    M.malha[141 + 169 + 169].set_obs_true();
    M.malha[141 + 3 * 169].set_obs_true();
    M.malha[141 + 4 * 169].set_obs_true();
    M.malha[141 + 5 * 169].set_obs_true();
    M.malha[141 + 6 * 169].set_obs_true();

    M.malha[144].set_obs_true();
    M.malha[144 + 169].set_obs_true();
    M.malha[144 + 169 + 169].set_obs_true();
    M.malha[144 + 3 * 169].set_obs_true();
    M.malha[144 + 4 * 169].set_obs_true();
    M.malha[144 + 5 * 169].set_obs_true();
    M.malha[144 + 6 * 169].set_obs_true();

    M.malha[145].set_obs_true();
    M.malha[145 + 169].set_obs_true();
    M.malha[145 + 169 + 169].set_obs_true();
    M.malha[145 + 3 * 169].set_obs_true();
    M.malha[145 + 4 * 169].set_obs_true();
    M.malha[145 + 5 * 169].set_obs_true();
    M.malha[145 + 6 * 169].set_obs_true();

    M.malha[146].set_obs_true();
    M.malha[146 + 169].set_obs_true();
    M.malha[146 + 169 + 169].set_obs_true();
    M.malha[146 + 3 * 169].set_obs_true();
    M.malha[146 + 4 * 169].set_obs_true();
    M.malha[146 + 5 * 169].set_obs_true();
    M.malha[146 + 6 * 169].set_obs_true();

    M.malha[147].set_obs_true();
    M.malha[147 + 169].set_obs_true();
    M.malha[147 + 169 + 169].set_obs_true();
    M.malha[147 + 3 * 169].set_obs_true();
    M.malha[147 + 4 * 169].set_obs_true();
    M.malha[147 + 5 * 169].set_obs_true();
    M.malha[147 + 6 * 169].set_obs_true();

    M.malha[148].set_obs_true();
    M.malha[148 + 169].set_obs_true();
    M.malha[148 + 169 + 169].set_obs_true();
    M.malha[148 + 3 * 169].set_obs_true();
    M.malha[148 + 4 * 169].set_obs_true();
    M.malha[148 + 5 * 169].set_obs_true();
    M.malha[148 + 6 * 169].set_obs_true();

    M.malha[149].set_obs_true();
    M.malha[149 + 169].set_obs_true();
    M.malha[149 + 169 + 169].set_obs_true();
    M.malha[149 + 3 * 169].set_obs_true();
    M.malha[149 + 4 * 169].set_obs_true();
    M.malha[149 + 5 * 169].set_obs_true();
    M.malha[149 + 6 * 169].set_obs_true();
    
    //Vizinhos sempre são <posicao do vetor> + 1 / <pv> - 1 / <pv> + (2* n_lat +1) / <pv> - (2*n_lat + 1).
    //Drone começa na posicao do vetor (tamanho do vetor)/2 ou seja em x,y,z = 0,0,0.
    //M.print_malha();

    do{
        if(pos == -1){
            pos = M.get_malha().size()/2;
            M.malha[pos].set_peso(0); //Define o peso do primeiro ponto como 0.
            distance = dist_2pts(M.malha[pos], destiny);
            M.malha[pos].set_real_dist(distance);
        }
        else{

            cout << "Ponto sendo olhado: " << endl;
            M.malha[pos].print_point();
            cout << endl;

           cout << "Vizinhos: " << endl;

            //RELAXAMENTO DOS VIZINHOS?
            pos_aux = pos - 1; //ponto "a esquerda do drone"
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso(); //esse e o peso certo?
                distance_aux = dist_2pts(M.malha[pos_aux], destiny); //esse auxiliar precisa?
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){ //pq esse realdist?

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + 1; //ponto "a direita do drone"
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2 + 1); //ponto "atras do drone"
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2 + 1); //ponto "a frente do drone"
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2); //ponto "esquerda-atrás do drone"
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2 + 2); //ponto "direita-atrás do drone"
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2); //ponto "esquerda-frente do drone"
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2 + 2); //ponto "direita-frente do drone"
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2 + 1)*(M.get_N_lon()*2 + 1); //ponto acima do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2 + 2)*(M.get_N_lon()*2 + 1);//ponto acima e frente do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2 + 2)*(M.get_N_lon()*2 + 1) + 1; //ponto acima e diagonal frente-direita do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2 + 2)*(M.get_N_lon()*2 + 1) - 1; //ponto acima e diagonal frente-esquerda do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2)*(M.get_N_lon()*2 + 1); //ponto acima e atrás do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2)*(M.get_N_lon()*2 + 1) + 1; //ponto acima e giagonal atrás-direita do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2)*(M.get_N_lon()*2 + 1) - 1; //ponto acima e diagonal atrás-esquerda do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2 + 1)*(M.get_N_lon()*2 + 1) + 1; //ponto acima e direita do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos + (M.get_N_lat()*2 + 1)*(M.get_N_lon()*2 + 1) - 1; //ponto acima e esquerda do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2 + 1)*(M.get_N_lon()*2 + 1);//ponto abaixo do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2 + 1)*(M.get_N_lon()*2 + 1) + 1; //ponto abaixo e direita do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2 + 1)*(M.get_N_lon()*2 + 1) - 1; //ponto abaixo e esquerda do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2 + 2)*(M.get_N_lon()*2 + 1); //ponto abaixo e atrás do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2 + 2)*(M.get_N_lon()*2 + 1) + 1; //ponto abaixo e atrás-direita do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2 + 2)*(M.get_N_lon()*2 + 1) - 1; //ponto abaixo e atrás-esquerda do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2)*(M.get_N_lon()*2 + 1); //ponto abaixo e frente do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2)*(M.get_N_lon()*2 + 1) + 1; //ponto abaixo e frente-direita do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            pos_aux = pos - (M.get_N_lat()*2)*(M.get_N_lon()*2 + 1) - 1; //ponto abaixo e frente-esquerda do drone
            if(pos_aux >= 0 && pos_aux <= M.malha.size() && M.malha[pos_aux].get_fechado() != 1 && M.malha[pos_aux].get_obs() != 1){
                dist_pts = dist_2pts(M.malha[pos], M.malha[pos_aux]) + M.malha[pos].get_peso();
                distance_aux = dist_2pts(M.malha[pos_aux], destiny);
				if((dist_pts + /*M.malha[pos].get_real_dist()*/ distance_aux) < (M.malha[pos_aux].get_peso() + M.malha[pos_aux].get_real_dist())){

                    M.malha[pos_aux].print_point();

					M.malha[pos_aux].set_peso(dist_pts); //Colocando o peso
					M.malha[pos_aux].set_ponto(pos); //Colocando o ponto. Utilizar o inteiro equivalente na malha.
					M.malha[pos_aux].set_real_dist(distance_aux);				}
            }

            M.malha[pos].set_fechado(); //fechando o ponto

            //ele ta fazendo copia aqui????//dapra melhorar esse algoritmo
            pos = pega_pos(M.malha, pos); //Agora ele olha só na malha, procurando pelo menor peso e se é fechado ou não

            cout << "próximo ponto a ser olhado: " << endl;
            M.malha[pos].print_point();
            cout << endl;


            //M.malha[pos].print_point();

            distance = dist_2pts(M.malha[pos], destiny);

            cout << distance << endl;

            //system("pause");
           // cout << endl;

            //cout << distance << endl;

            //Condição de parada. Checar sobre distance < 8.
            if (distance < 8){
				arrived = 1;
			}

			//dijkstra_list.clear();
        }
    }while(arrived == 0);

    //Organizar o vetor de pontos, começando do destino e "retraçando" o caminho até o início.
    int pos_aux2 = -1;
    do{
        points_list.push_back(M.malha[pos]);
        pos = M.malha[pos].get_pto_ant();
        pos_aux2 = pos;
    }while(pos_aux2 != M.malha.size()/2);

    for (Point it : M.get_malha()) {
        if (it.get_obs() == 1) {
            points_list.push_back(it);
        }
    }

    return points_list;
}
