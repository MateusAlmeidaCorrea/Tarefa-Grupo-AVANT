#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <vector>

std::vector<double> pegarpontos(int argc, char *argv[]);
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
geometry_msgs::PoseStamped gps;
void gps_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    gps = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    //publisher destino
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    //cliente para realizar o armamento
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    //cliente para o voo
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    //subscriber para o gps
    ros::Subscriber gps_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 100, 	 gps_cb);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //armando o drone para conseguir voar
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if( arming_client.call(arm_cmd) && arm_cmd.response.success){
        ROS_INFO("Vehicle armed");
    }
    //coordenadas para onde o drone deve ir
    std::vector<double> pontos = pegarpontos(argc, argv);
    std::vector<double> pontos_final;
    for (int j = pontos.size()-1; j >= 0; j--){
	pontos_final.push_back(pontos[j]);
	ROS_INFO("%f", pontos[j]);
    }
    geometry_msgs::PoseStamped pose;
    //posicao inicial
    pose.pose.position.x = 0.8; //ajuste feito para o drone ir para o centro do quadrado
    pose.pose.position.y = -0.5;//por causa do ruido esse ajusto nao foi tao exato
    pose.pose.position.z = 1;//ajuste feito pois no codigo o drone inicia-se no meio do espaco e para esse codigo ele deve comecar na altura mais baixa
    for(int i = 100; ros::ok() && i > 0; --i){
      rate.sleep();
    }
    //preparando o drone para decolar
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
        ROS_INFO("Offboard enabled");
    }
    //loop para a atualizacao das posicoes
    int i = 0;
    int intervalo = pontos_final.size() - 4; // intervalo feito pois o i deve atualizar uma vez a menos em relacao a quantidade de pontos existentes
    while(ros::ok()){
	//olhar se o drone ja chegou na posicao certa para atualizar o destino
	if(((pose.pose.position.x - 0.2) < gps.pose.position.x && gps.pose.position.x < (pose.pose.position.x + 0.2)) &&
	   ((pose.pose.position.y - 0.2) < gps.pose.position.y && gps.pose.position.y < (pose.pose.position.y + 0.2)) &&
	   ((pose.pose.position.z - 0.2) < gps.pose.position.z && gps.pose.position.z < (pose.pose.position.z + 0.2))){
	   // condicao que atualiza a prosicao do drone caso ainda existam pontos para serem seguidos
	   if (i < intervalo){
	     ROS_INFO("Mudando o ponto");
	     pose.pose.position.x = (pontos_final[i+2]/15+0.7);//ajuste para o centro e para o ruido do drone
	     pose.pose.position.y = (pontos_final[i+1]/15-0.4);
             pose.pose.position.z = (pontos_final[i]/15+3);
	     i = i+3;	
	   }
	   else{
	     pose.pose.position.x = (pontos_final[i+2]/15+0.7);
	     pose.pose.position.y = (pontos_final[i+1]/15-0.4);
             pose.pose.position.z = (pontos_final[i]/15+3);	
	   }
    	}
	//publica a posicao que o drone deve ir
	local_pos_pub.publish(pose);
	//faz essa requisicao acontecer na hora
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


