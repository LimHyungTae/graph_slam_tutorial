# PCL cheat sheet(1/2)

Original author: Hyungtae Lim (shapelim@kaist.ac.kr)   


----------------

## PCL 선언하는 법 & T Type

### T Type

pcl상의 PointCloud pcl::PointCloud<T>에는 다양한 type을 담을 수 있는데,

주로 **LiDAR**를 사용할 때는 *pcl::PointXYZ, pcl::PointXYZI*를 많이 사용한다.

**RGB-D나 스테레오 카메라**는 depth를 image에 align시키면 point의 색깔도 알 수 있기 때문에 *pcl::PointXYZRGB(A)*를 사용하는 경우도 있다.

더 다양한 type은 원래 pcl tutorial 페이지에서 확인할 수 있다. <br/>
http://www.pointclouds.org/documentation/tutorials/adding_custom_ptype.php#adding-custom-ptype

```cpp
pcl::PointCloud<pcl::PointXYZ> cloud;
pcl::PointCloud<pcl::PointXYZI> cloud;
pcl::PointCloud<pcl::PointNormal> cloud;
```

{% highlight ruby %}
pcl::PointCloud<pcl::PointXYZ> cloud;
pcl::PointCloud<pcl::PointXYZI> cloud;
pcl::PointCloud<pcl::PointNormal> cloud;
{% endhighlight %}

```cpp
pcl::PointXYZ point_xyz;
point_xyz.x = 1;
point_xyz.y = 2;
point_xyz.z = 3;
```
혹은 아래와 같이 한 줄로 선언할 수도 있다.
```cpp 

pcl::PointXYZ point_xyz = {1, 2, 3}; // 1, 2, 3이 각각 x, y, z로 지정된다.
```

아래의 예시들은 다음과 같이 header file과 namespace가 선언되어 있다고 가정한다.
```cpp
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

using namespace std;
```

### pcl::PointCloud 선언해서 Points에 Point 넣는 법

기본적으로 pcl은 std::vector의 사용법과 유사하다.

왜냐하면 pcl의 내부를 살펴보면 std::vector로 구성되어 있기 때문이다 (http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html 참조)

```cpp
pcl::PointCloud<pcl::PointXYZ> cloud;
cloud.resize(3); //cloud의 size를 3으로 설정 

cloud.points[0].x = 1;
cloud.points[0].y = 2;
cloud.points[0].z = 3;

cloud.points[1].x = 4;
cloud.points[1].y = 5;
cloud.points[1].z = 6;

cloud.points[2].x = 7;
cloud.points[2].y = 8;
cloud.points[2].z = 9;

```

or
```cpp
pcl::PointCloud<pcl::PointXYZ> cloud;
pcl::PointXYZ point_xyz; // pcl::PointXYZ이라는 type에 data를 담는다.

point_xyz.x = 1;
point_xyz.y = 2;
point_xyz.z = 3;
cloud.push_back(point_xyz);

point_xyz.x = 4;
point_xyz.y = 5;
point_xyz.z = 6;
cloud.push_back(point_xyz);

point_xyz.x = 7;
point_xyz.y = 8;
point_xyz.z = 9;
cloud.push_back(point_xyz);
```
or
```cpp
pcl::PointCloud<pcl::PointXYZ> cloud;
cloud.push_back(pcl::PointXYZ(1, 2, 3));
cloud.push_back(pcl::PointXYZ(4, 5, 6));
cloud.push_back(pcl::PointXYZ(7, 8, 9));
```

출력을 하면 아래와 같은 결과를 볼 수 있다.
```cpp
for (int i = 0 ; i < cloud.size(); ++i){
      cout << i << ": ";
      cout << cloud.points[i].x << ", ";
      cout << cloud.points[i].y << ", ";
      cout << cloud.points[i].z << endl;
      
  }
```
##### Result: <br/>
0: 1, 2, 3 <br/> 
1: 4, 5, 6 <br/>
2: 7, 8, 9 

##  Functions

http://docs.pointclouds.org/1.8.1/classpcl_1_1_point_cloud.html

### begin(): PointCloud의 첫 부분을 가르키는 *iterator*를 반환함

```cpp
cout << "begin(): ";
cout << cloud.begin()->x << ", ";
cout << cloud.begin()->y << ", ";
cout << cloud.begin()->z << endl;
```

->의 의미는 옆 사이트에 설명되어 있다. https://ianuarias.tistory.com/16 

->의 의미 한 줄 요약: return 값이 주소값이기 때문에, 주소값이 가르키는 객체의 멤버변수/멤버함수는 ->로 지칭함.

##### Result::<br/>
begin(): 1, 2, 3

### end(): PointCloud의 끝 부분을 가르키는 *iterator*를 반환함

```cpp
cout << "end(): ";
cout << cloud.end()->x << ", ";
cout << cloud.end()->y << ", ";
cout << cloud.end()->z << endl;
```
##### Result: <br/>
end(): 0, 0, 0

????? 왜 0, 0, 0?: vector의 end()처럼 마지막 요소의 다음 부분을 가르키는 iterator를 리턴하기 때문.

따라서 우리가 원하는 pcl::PointCloud의 제일 뒷쪽의 요소를 가르키는 iterator를 뜻하려면 다음과 같이 -1을 빼주면 된다.

```cpp
cout << "end() -1: ";
cout << (cloud.end()-1)->x << ", ";
cout << (cloud.end()-1)->y << ", ";
cout << (cloud.end()-1)->z << endl;
```
##### Result: <br/>
end() - 1: 7, 8, 9

### front(): 첫번째 *원소*를 반환함
```cpp
cout << "front(): ";
cout << cloud.front().x << ", ";
cout << cloud.front().y << ", ";
cout << cloud.front().z << endl;
```
##### Result: <br/>
front(): 1, 2, 3

### back(): 마지막 *요소*를 반환함

```cpp
cout << "back(): ";
cout << cloud.back().x << ", ";
cout << cloud.back().y << ", ";
cout << cloud.back().z << endl;
```

##### Result: <br/>
back(): 7, 8, 9

### at(int index)
pcl::PointCloud 상의 index에 해당하는 요소를 반환함.

```cpp
cout << "at(1): ";
cout << cloud.at(1).x << ", ";
cout << cloud.at(1).y << ", ";
cout << cloud.at(1).z << endl;
```
##### Result: <br/>
at(1): 4, 5, 6

### empty()
```cpp
if (cloud.empty()) cout << "True"; else cout << "False";
cout << " | size of pc: " << cloud.size() << endl;
```
##### Result: <br/>
False | size of pc: 3

### clear()
```cpp
cloud.empty();
if (cloud.empty()) cout << "True"; else cout << "False";
cout << " | size of pc: " << cloud.size() << endl;
```
##### Result: <br/>
True | size of pc: 0


## ★PointCloud 합치기
~~이런 pythonic한 문법이 되다니...갓PCL...~~
```cpp
pcl::PointCloud<pcl::PointXYZ> cloud2;
cloud2.push_back(pcl::PointXYZ(1, 2, 3));
cloud2.push_back(pcl::PointXYZ(4, 5, 6));


pcl::PointCloud<pcl::PointXYZ> cloud3;
cloud3.push_back(pcl::PointXYZ(7, 8, 9));
cloud3.push_back(pcl::PointXYZ(10, 11, 12));

cloud2 += cloud3;

cout <<"size: " << cloud2.size() << endl;
for (int i = 0 ; i < cloud2.size(); ++i){
   cout << i << ": ";
   cout << cloud2.points[i].x << ", ";
   cout << cloud2.points[i].y << ", ";
   cout << cloud2.points[i].z << endl;
}
```
##### Result: <br/>
size: 4 <br/>
0: 1, 2, 3 <br/>
1: 4, 5, 6 <br/>
2: 7, 8, 9 <br/>
3: 10, 11, 12

## PCL pointer Ptr 선언

### 사용하는 이유

pcl::PointCloud의 pointer는 아래와 같이 선언할 수 있다.

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
```

기존에 우리가 알던 &#42로 pointer를 선언하는 것과 다르다.

하지만 pcl에 구현되어있는 함수들이 대부분 Ptr을 매개체로 하여 출력값을 저장하기 때문에, 사용해야 한다.

https://github.com/LimHyungTae/LimHyungTae.github.io/blob/master/_posts/2019-11-29-%5BROS%5D%20PCL%20%EC%9E%90%EC%A3%BC%20%EC%93%B0%EC%9D%B4%EB%8A%94%20%EA%B2%83%20%EC%A0%95%EB%A6%AC.md

그럼 pcl::PointCloud의 주소를 Ptr로 할당시키려면 아래와 같이 해야할까?
```cpp
ptr_cloud = &cloud2;
```

아님!! ~~boost::shared_ptr랑 관련 있는 거 같은데 c++ 고수님들 왜 그런건지 아시는 분 가르쳐주세요...~~

아래와 같이 하면 pcl::PointCloud를 Ptr에 할당할 수 있다.

```cpp
*ptr_cloud = cloud2;
```

### 사용법

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
*ptr_cloud = cloud2;

cout<<"Original: "<<endl;
for (int i = 0 ; i < cloud3.size(); ++i){
cout << i << ": ";
cout << cloud3.points[i].x << ", ";
cout << cloud3.points[i].y << ", ";
cout << cloud3.points[i].z << endl;
}

cloud3 = *ptr_cloud;

cout << "After: " << endl;
//  cout<<"size: " << cloud3.size() << endl;
for (int i = 0 ; i < cloud3.size(); ++i){
cout << i << ": ";
cout << cloud3.points[i].x << ", ";
cout << cloud3.points[i].y << ", ";
cout << cloud3.points[i].z << endl;
}
```
##### Result: <br/>
Original: <br/>
0: 7, 8, 9 <br/>
1: 10, 11, 12 <br/>
After: <br/> 
0: 1, 2, 3 <br/>
1: 4, 5, 6 <br/>
2: 7, 8, 9 <br/>
3: 10, 11, 12

### +=는 값을 할당하는 걸까, 복사하는 걸까?

```cpp
cloud3.push_back(pcl::PointXYZ(12, 13, 14));
cout<<"After: "<<endl;
  for (int i = 0 ; i < cloud2.size(); ++i){
  cout<<i<<": ";
  cout<<cloud2.points[i].x<<", ";
  cout<<cloud2.points[i].y<<", ";
  cout<<cloud2.points[i].z<<endl;
  }
}
```
##### Result: <br/>
After: <br/> 
0: 1, 2, 3 <br/>
1: 4, 5, 6 <br/>
2: 7, 8, 9 <br/>
3: 10, 11, 12

그대로임을 알 수 있다. 즉, 주소를 할당받아 link되어 있지 않고, points들을 통째로 복사해오는 것임.



