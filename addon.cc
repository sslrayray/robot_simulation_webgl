#include <math.h>
#include <node.h>
#include <v8.h>


#include <test.h>
#include <Eigen/Core>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>
#include <time.h> 

using namespace std;
using namespace Eigen;
using namespace v8;

#define R_PI 3.14159265359

int g_nodes_count = 0;
Local<v8::Array> g_arr_min_x;
Local<v8::Array> g_arr_min_y;
Local<v8::Array> g_arr_min_z;

Local<v8::Array> g_arr_max_x;
Local<v8::Array> g_arr_max_y;
Local<v8::Array> g_arr_max_z;

int kd_main();
double g_axis_d[6];
int g_collision_flag = 0;
MatrixXd t0(4, 1);


Isolate* isolate;
void CreateObject(const FunctionCallbackInfo<Value>& args) {
  isolate = Isolate::GetCurrent();
  HandleScope scope(isolate);

  Local<Object> obj = Object::New(isolate);
  
  for (int i = 0; i < 6; i++)
  {
	  std::string::size_type sz;
	  v8::String::Utf8Value param1(args[i]->ToString());
	  std::string str1 = std::string(*param1);
	  g_axis_d[i] = std::stod (str1,&sz);
	  //cout << g_axis_d[i] << " ";
  }
  //cout << endl;

  //string str = str1 + std::string(", ") + str2;
  //Local<String> lstr = v8::String::NewFromUtf8(isolate, str.c_str());

  
  
  g_arr_min_x = v8::Array::New(isolate);
  g_arr_min_y = v8::Array::New(isolate);
  g_arr_min_z = v8::Array::New(isolate);
         
  g_arr_max_x = v8::Array::New(isolate);
  g_arr_max_y = v8::Array::New(isolate);
  g_arr_max_z = v8::Array::New(isolate);

  obj->Set(String::NewFromUtf8(isolate, "arr_min_x"), g_arr_min_x);
  obj->Set(String::NewFromUtf8(isolate, "arr_min_y"), g_arr_min_y);
  obj->Set(String::NewFromUtf8(isolate, "arr_min_z"), g_arr_min_z);
  
  obj->Set(String::NewFromUtf8(isolate, "arr_max_x"), g_arr_max_x);
  obj->Set(String::NewFromUtf8(isolate, "arr_max_y"), g_arr_max_y);
  obj->Set(String::NewFromUtf8(isolate, "arr_max_z"), g_arr_max_z);

  Matrix3d tt;
  tt << 0, 0, 0, 1, 1, 1, 1, 1, 1;
  //double tt = sin(3.1415926/2);
  
  g_nodes_count = 0;
  kd_main();
  obj->Set(String::NewFromUtf8(isolate, "nodes_count"), Number::New(isolate,g_nodes_count));
  obj->Set(String::NewFromUtf8(isolate, "collision_flag"), Number::New(isolate,g_collision_flag));
  obj->Set(String::NewFromUtf8(isolate, "g_x"), Number::New(isolate,t0(0,0)));
  obj->Set(String::NewFromUtf8(isolate, "g_y"), Number::New(isolate,t0(1,0)));
  obj->Set(String::NewFromUtf8(isolate, "g_z"), Number::New(isolate,t0(2,0)));
  
  args.GetReturnValue().Set(obj);
}

void Init(Handle<Object> exports, Handle<Object> module) {
  NODE_SET_METHOD(module, "exports", CreateObject);
}












// KDtree.cpp: 定义控制台应用程序的入口点。
//



#define MAX_NUM 1000000

class Point
{
public:
	double x;
	double y;
	double z;
	Point(double x_t, double y_t, double z_t) :x(x_t), y(y_t), z(z_t) {};
	Point() :x(0), y(0), z(0) {};
};

class Kd_Node
{
public:
	vector<Point*> point_list;
	Point min_margin;
	Point max_margin;
	Kd_Node* left_child;
	Kd_Node* right_child;
	int dim_no;
	int middle_value;

	Kd_Node() :min_margin(MAX_NUM, MAX_NUM, MAX_NUM), max_margin(-MAX_NUM, -MAX_NUM, -MAX_NUM), left_child(NULL), right_child(NULL), dim_no(0), middle_value(0) {}
	~Kd_Node() {
		point_list.clear();
	};

	void set_best_dim_no()
	{
		double x_val_total = 0, y_val_total = 0, z_val_total = 0;
		double x_val_avg = 0, y_val_avg = 0, z_val_avg = 0;
		double x_val_var = 0, y_val_var = 0, z_val_var = 0;

		for (vector<Point*>::iterator it = point_list.begin(); it != point_list.end(); ++it)
		{
			x_val_total += (*it)->x;
			y_val_total += (*it)->y;
			z_val_total += (*it)->z;
		}

		x_val_avg = x_val_total / point_list.size();
		y_val_avg = y_val_total / point_list.size();
		z_val_avg = z_val_total / point_list.size();

		for (vector<Point*>::iterator it = point_list.begin(); it != point_list.end(); ++it)
		{
			x_val_var += ((*it)->x - x_val_avg) * ((*it)->x - x_val_avg);
			y_val_var += ((*it)->y - y_val_avg) * ((*it)->y - y_val_avg);
			z_val_var += ((*it)->z - z_val_avg) * ((*it)->z - z_val_avg);
		}

		if ((x_val_var > y_val_var) && (x_val_var > z_val_var))
			dim_no = 0;

		if ((y_val_var > x_val_var) && (y_val_var > z_val_var))
			dim_no = 1;

		if ((z_val_var > x_val_var) && (z_val_var > y_val_var))
			dim_no = 2;
	}

	void update_local_min_max()
	{
		for (vector<Point*>::iterator it = point_list.begin(); it != point_list.end(); ++it)
		{
			min_margin.x = ((*it)->x < min_margin.x ? (*it)->x : min_margin.x);
			min_margin.y = ((*it)->y < min_margin.y ? (*it)->y : min_margin.y);
			min_margin.z = ((*it)->z < min_margin.z ? (*it)->z : min_margin.z);

			max_margin.x = ((*it)->x > max_margin.x ? (*it)->x : max_margin.x);
			max_margin.y = ((*it)->y > max_margin.y ? (*it)->y : max_margin.y);
			max_margin.z = ((*it)->z > max_margin.z ? (*it)->z : max_margin.z);
		}
	}
};

class Obstacle
{
public:
	Obstacle(){
		min_x = 0;
		min_y = 0;
		min_z = 0;
		max_x = 0;
		max_y = 0;
		max_z = 0;
	}

	Obstacle(double min_x_t, double min_y_t, double min_z_t, double max_x_t, double max_y_t, double max_z_t){
		min_x = min_x_t;
		min_y = min_y_t;
		min_z = min_z_t;
		max_x = max_x_t;
		max_y = max_y_t;
		max_z = max_z_t;
	}

	bool check_collision(Point* min_margin, Point* max_margin)
	{
		//x
		if (min_x > max_margin->x) {
			//cout << "1" << endl;
			return false;
		}
		if (max_x < min_margin->x) {
			//cout << "2" << endl;
			return false;
		}
		//y
		if (min_y > max_margin->y) {
			//cout << "3" << endl;
			return false;
		}
		if (max_y < min_margin->y) {
			//cout << "max y " << max_y << "min_margin y" << min_margin->y << endl;
			//cout << "4" << endl;
			return false;
		}
		//z
		if (min_z > max_margin->z) {
			//cout << "5" << endl;
			return false;
		}
		if (max_z < min_margin->z) {
			//cout << "6" << endl;
			return false;
		}
		return true;
	}

	double min_x;
	double min_y;
	double min_z;

	double max_x;
	double max_y;
	double max_z;
};

Obstacle g_obstacle(100-25, 0-25, 0-25, 100+25, 0+25, 0+25);

bool node_compare_x(Point* a, Point* b) { return ((a->x) < (b->x)); }
bool node_compare_y(Point* a, Point* b) { return ((a->y) < (b->y)); }
bool node_compare_z(Point* a, Point* b) { return ((a->z) < (b->z)); }


void construct_Kd_tree(Kd_Node* kd_node, int depth)
{
	if (kd_node->point_list.size() == 0)
	{
		return;
	}

	//update min & max node
	kd_node->update_local_min_max();

	//update middle value
	int middle_no = 0;
	middle_no = kd_node->point_list.size() / 2;
	//set dimension with largest variance
	kd_node->set_best_dim_no();
	if (kd_node->dim_no == 0)
	{
		sort(kd_node->point_list.begin(), kd_node->point_list.end(), node_compare_x);
		kd_node->middle_value = kd_node->point_list.at(middle_no)->x;

		//cout << "sort by x" << endl;
	}
	else if (kd_node->dim_no == 1)
	{
		sort(kd_node->point_list.begin(), kd_node->point_list.end(), node_compare_y);
		kd_node->middle_value = kd_node->point_list.at(middle_no)->y;
		//cout << "sort by y" << endl;
	}
	else {
		sort(kd_node->point_list.begin(), kd_node->point_list.end(), node_compare_z);
		kd_node->middle_value = kd_node->point_list.at(middle_no)->z;
		//cout << "sort by z" << endl;
	}

	if (kd_node->point_list.size() <= 2)
	{
		return;
	}

	if (depth == 0)
	{
		g_arr_min_x->Set(g_nodes_count, Number::New(isolate, kd_node->min_margin.x));
		g_arr_min_y->Set(g_nodes_count, Number::New(isolate, kd_node->min_margin.y));
		g_arr_min_z->Set(g_nodes_count, Number::New(isolate, kd_node->min_margin.z));
		//cout << "var p_min = new THREE.Vector3(" << kd_node->min_margin.x << ", " << kd_node->min_margin.y << ", " << kd_node->min_margin.z << ");" << endl;
		//cout << "var p_max = new THREE.Vector3(" << kd_node->max_margin.x << ", " << kd_node->max_margin.y << ", " << kd_node->max_margin.z << ");" << endl;
		//cout << "scene_add_line_box(p_min, p_max);" << endl;
		
		//cout <<  kd_node->min_margin.y << " " << kd_node->max_margin.y;

		g_arr_max_x->Set(g_nodes_count, Number::New(isolate, kd_node->max_margin.x));
		g_arr_max_y->Set(g_nodes_count, Number::New(isolate, kd_node->max_margin.y));
		g_arr_max_z->Set(g_nodes_count, Number::New(isolate, kd_node->max_margin.z));

		g_nodes_count++;
		
		if (g_obstacle.check_collision(&kd_node->min_margin, &kd_node->max_margin)) {
			g_collision_flag = 1;
		}
		return;
	}

	//if vector list contains more than 2 nodes, continue with dfs
	kd_node->left_child = new Kd_Node;
	for (int i = 0; i <= middle_no; i++)
	{
		kd_node->left_child->point_list.push_back(kd_node->point_list.at(i));
	}

	construct_Kd_tree(kd_node->left_child, depth - 1);
	

	kd_node->right_child = new Kd_Node;
	for (int i = middle_no; i < kd_node->point_list.size(); i++)
	{
		kd_node->right_child->point_list.push_back(kd_node->point_list.at(i));
	}
	construct_Kd_tree(kd_node->right_child, depth - 1);
}

void free_Kd_tree(Kd_Node* kd_node)
{
	if (kd_node->left_child == NULL || kd_node->right_child == NULL) {
		return;
	}
	free_Kd_tree(kd_node->left_child);
	free_Kd_tree(kd_node->right_child);
	delete(kd_node->left_child);
	delete(kd_node->right_child);
	kd_node->left_child = NULL;
	kd_node->right_child = NULL;
	
}

int kd_main()
{
	 double start,end,cost;    
    
	static Kd_Node* root_link[6], *root_current = NULL;
	static int first_call_main_flag = 1;
	double pos_x_min = MAX_NUM;
	double pos_y_min = MAX_NUM;
	double pos_z_min = MAX_NUM;

	double pos_x_max = -MAX_NUM;
	double pos_y_max = -MAX_NUM;
	double pos_z_max = -MAX_NUM;

	start=clock(); 

	if (first_call_main_flag == 1)
	{
		for (int i = 0; i < 6; i++)
		{
			stringstream ostr;
			ostr << string("D:\\courses\\robotics_monipulation\\term_prj\\base_pts") << (i) <<  string(".txt");
			string filename = ostr.str(); 
			cout << filename << endl;
			ifstream file_in(filename);

			root_link[i] = new Kd_Node();
			if (file_in.is_open())
			{
				double pos_x, pos_y, pos_z;
				while ((file_in >> pos_x >> pos_y >> pos_z))
				{
					pos_x_min = (pos_x < pos_x_min ? pos_x : pos_x_min);
					pos_y_min = (pos_y < pos_y_min ? pos_y : pos_y_min);
					pos_z_min = (pos_z < pos_z_min ? pos_z : pos_z_min);

					pos_x_max = (pos_x > pos_x_max ? pos_x : pos_x_max);
					pos_y_max = (pos_y > pos_y_max ? pos_y : pos_y_max);
					pos_z_max = (pos_z > pos_z_max ? pos_z : pos_z_max);

					root_link[i]->point_list.push_back(new Point(pos_x, pos_y, pos_z));
					
				}
			}
			file_in.close();
		}

		


		
		first_call_main_flag = 0;
	}
	//cout << "min :" << pos_x_min << " " << pos_y_min << " " << pos_z_min << endl;
	//cout << "max :" << pos_x_max << " " << pos_y_max << " " << pos_z_max << endl;

	
	//update points coordinate after rotation
	MatrixXd ksi[6];
	MatrixXd R[6];
	MatrixXd P[6];
	MatrixXd G[6];
	double theta[6];

	ksi[0] = MatrixXd(6, 1);
	ksi[1] = MatrixXd(6, 1);
	ksi[2] = MatrixXd(6, 1);
	ksi[3] = MatrixXd(6, 1);
	ksi[4] = MatrixXd(6, 1);
	ksi[5] = MatrixXd(6, 1);

	G[0] = MatrixXd(4, 4);
	G[1] = MatrixXd(4, 4);
	G[2] = MatrixXd(4, 4);
	G[3] = MatrixXd(4, 4);
	G[4] = MatrixXd(4, 4);
	G[5] = MatrixXd(4, 4);

	
	t0(0, 0) = 0;
	t0(1, 0) = 213.5;
	t0(2, 0) = -15;
	t0(3, 0) = 1;

	R[0] = MatrixXd(3, 3);
	R[1] = MatrixXd(3, 3);
	R[2] = MatrixXd(3, 3);
	R[3] = MatrixXd(3, 3);
	R[4] = MatrixXd(3, 3);
	R[5] = MatrixXd(3, 3);

	P[0] = MatrixXd(3, 1);
	P[1] = MatrixXd(3, 1);
	P[2] = MatrixXd(3, 1);
	P[3] = MatrixXd(3, 1);
	P[4] = MatrixXd(3, 1);
	P[5] = MatrixXd(3, 1);

	theta[0] = g_axis_d[0];
	
	//theta[0] = R_PI /2;
	theta[1] = g_axis_d[1];
	theta[2] = g_axis_d[2];
	theta[3] = g_axis_d[3];
	theta[4] = g_axis_d[4];
	theta[5] = g_axis_d[5];
	

	ksi[0] << 0, 0, 0, 0, -1, 0;
	ksi[1] << 0, 15, 46.5, -1, 0, 0;	
	ksi[2] << 0, -15, -121.5, 1, 0, 0;
	ksi[3] << 15, 0, 0, 0, 1, 0;
	ksi[4] << 0, 15, 202, -1, 0, 0;
	ksi[5] << 15, 0, 0, 0, 1, 0;
 

	for (int i = 0; i < 6; i++)
	{
		Matrix3d w_hat;
		Matrix3d I = Matrix3d::Identity();
		Vector3d v;
		Vector3d w;
		v.x() = ksi[i](0, 0);
		v.y() = ksi[i](1, 0);
		v.z() = ksi[i](2, 0);
		w.x() = ksi[i](3, 0);
		w.y() = ksi[i](4, 0);
		w.z() = ksi[i](5, 0);
		w_hat << 0, -w.z(), w.y(), \
				w.z(), 0, -w.x(), \
				- w.y(), w.x(), 0;
		R[i] = I + w_hat * sin(theta[i]) + w_hat * w_hat * (1 - cos(theta[i]));
		G[i].topLeftCorner(3, 3) = R[i];
		G[i].bottomLeftCorner(1, 3).setZero();
		G[i](3, 3) = 1;
		P[i] = (I - R[i]) * (w_hat * v);
		G[i].topRightCorner(3, 1) = P[i];
	}

	for (int link_num = 0; link_num < 6; link_num++)
	{
		root_current = new Kd_Node();
		for (int i = 0; i < root_link[link_num]->point_list.size(); i++)
		{
			//homogeneous coordinate
			MatrixXd p(4, 1);
			MatrixXd p_new(4, 1);
			p(0, 0) = root_link[link_num]->point_list.at(i)->x;
			p(1, 0) = root_link[link_num]->point_list.at(i)->y;
			p(2, 0) = root_link[link_num]->point_list.at(i)->z;
			p(3, 0) = 1;
			for (int j = link_num; j >= 0; j--)
				p = G[j] * p;
			
			root_current->point_list.push_back(new Point(p(0, 0),  p(1, 0),  p(2, 0)));
		}
	
		//build K-d tree
		int clusters_count = 1;
		int expected_clusters = 8;
		int max_depth = round(log2(expected_clusters));
		
		construct_Kd_tree(root_current, max_depth);
		free_Kd_tree(root_current);
		for (vector<Point*>::iterator it = root_current->point_list.begin(); it != root_current->point_list.end(); ++it) 
			delete *it;
		root_current->point_list.clear();
	}
	
	for (int j = 5; j >= 0; j--)
		t0 = G[j] * t0;
	//free(root);
	//std::sort(root->point_list.begin(), root->point_list.end(), node_compare_z);

	//for (vector<Point*>::iterator it = root->point_list.begin(); it != root->point_list.end(); ++it)
	//{
	//	cout << (*it)->x << " " << (*it)->y << " " << (*it)->z << endl;
	//}

	//getchar();
	//file_in.close();

	end=clock();  

	cost=end-start;    
	printf("%f\n",cost); 
	return 0;
}

NODE_MODULE(addon, Init)