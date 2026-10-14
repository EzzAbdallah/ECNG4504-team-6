// Host-side simulation of the exploration algorithm on the real Maze_1.pdf graph.
#include "Arduino.h"
#include "maze_map.h"
#include <random>
#include <vector>

struct TNode { int x, y; bool circle; };       // true maze (pixels of the 750px-wide PDF render)
static const TNode T[] = {
  {75,75,true},{75,350,0},{285,350,0},{450,350,0},{675,350,0},{675,75,0},{450,75,0},{285,75,0},
  {285,500,0},{75,500,0},{75,800,0},{285,715,0},{450,715,0},{563,715,0},{563,540,0},{675,715,0},
  {285,925,0},{75,925,0},{450,925,0},{675,925,true}};
static const int NT = sizeof(T)/sizeof(T[0]);
static const int E[][2] = {{0,1},{1,2},{2,3},{3,4},{4,5},{5,6},{6,7},{7,2},{6,3},{2,8},{8,9},{9,10},
  {8,11},{11,12},{3,12},{12,13},{13,14},{13,15},{11,16},{16,17},{12,18},{18,19},{6,3}};
// (6,3) listed twice on purpose? remove duplicate below.
static int link_[20][4];
static int dirOf(int a,int b){ int dx=T[b].x-T[a].x, dy=T[b].y-T[a].y; if(dy<0)return 0; if(dx>0)return 1; if(dy>0)return 2; return 3; }

int main(int argc, char** argv) {
  double noise = argc>1 ? atof(argv[1]) : 0.05;
  int runs = argc>2 ? atoi(argv[2]) : 200;
  bool show = argc>3;
  int okRuns=0, dupRuns=0, overflowRuns=0, stuckRuns=0;
  for (int run=0; run<runs; run++) {
    std::mt19937 rng(run+1);
    std::normal_distribution<double> g(0.0, noise);
    for(int i=0;i<NT;i++) for(int d=0;d<4;d++) link_[i][d]=-1;
    int ne = sizeof(E)/sizeof(E[0]) - 1;      // skip the duplicate last entry
    for (int k=0;k<ne;k++){ int a=E[k][0], b=E[k][1]; link_[a][dirOf(a,b)]=b; link_[b][dirOf(b,a)]=a; }
    MazeMap m; m.reset(2);
    int trueNode=0; int8_t cur=0; uint8_t h=2; bool finished=false; int steps=0; bool overflow=false;
    // first move: south from start
    while (steps++ < 400) {
      int8_t nxt = m.planNext(cur, h);
      if (nxt < 0) { finished=true; break; }
      h = nxt;
      int to = link_[trueNode][h];
      if (to < 0) { printf("BUG: planner chose non-existent exit\n"); return 1; }
      double dist = abs(T[to].x-T[trueNode].x)+abs(T[to].y-T[trueNode].y);
      double ticks = dist*8.0*(1.0+g(rng));          // 1 px = 8 ms of travel
      uint8_t exits=0; for(int d=0;d<4;d++) if(link_[to][d]>=0) exits|=1<<d;
      Arrival a = m.arrive(cur, h, (uint32_t)ticks, exits, T[to].circle);
      if (a.overflow) { overflow=true; break; }
      cur = a.id; trueNode = to;
    }
    bool good = finished && m.count()==NT && cur==0;
    if (good) okRuns++; else if (overflow) overflowRuns++; else if(!finished) stuckRuns++; else dupRuns++;
    if (show && run==0) { Print p; m.print(p); }
  }
  printf("noise=%.0f%%  runs=%d  perfect=%d  duplicated-nodes=%d  overflow=%d  stuck=%d\n",
         noise*100, runs, okRuns, dupRuns, overflowRuns, stuckRuns);
  return 0;
}
