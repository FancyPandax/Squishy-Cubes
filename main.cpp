#include "raylib.h"
#include <vector>
#include <cmath>
#include <limits>
#include <cstdlib>

// ---------------- Vec2 ----------------
struct Vec2 {
    float x, y;
    Vec2(float _x=0,float _y=0):x(_x),y(_y){}
    Vec2 operator+(const Vec2& rhs) const { return Vec2(x+rhs.x,y+rhs.y);}
    Vec2 operator-(const Vec2& rhs) const { return Vec2(x-rhs.x,y-rhs.y);}
    Vec2 operator*(float f) const { return Vec2(x*f,y*f);}
    Vec2 operator/(float f) const { return Vec2(x/f,y/f);}
    Vec2& operator+=(const Vec2& rhs){x+=rhs.x;y+=rhs.y;return *this;}
    Vec2& operator-=(const Vec2& rhs){x-=rhs.x;y-=rhs.y;return *this;}
    Vec2& operator*=(float f){x*=f;y*=f;return *this;}
    Vec2& operator/=(float f){x/=f;y/=f;return *this;}
    float Length() const {return sqrtf(x*x+y*y);}
    Vec2 Normalized() const {float len=Length(); return len==0?Vec2(0,0):Vec2(x/len,y/len);}
};

// ---------------- Particle ----------------
struct Particle{
    Vec2 position;
    Vec2 oldPosition;
    Vec2 acceleration;
    bool pinned=false;
    Particle(Vec2 pos):position(pos),oldPosition(pos),acceleration(0,0),pinned(false){}
};

// ---------------- Stick ----------------
struct Stick{
    Particle* p1;
    Particle* p2;
    float restLength;
    Stick(Particle* a,Particle* b,float len):p1(a),p2(b),restLength(len){}
};

// ---------------- Quad ----------------
struct Quad{
    Particle* p0; Particle* p1; Particle* p2; Particle* p3;
    Vec2 offset0,offset1,offset2,offset3;
};

// ---------------- Global Variables ----------------
float DIST_STIFF=0.3f;
float SHAPE_STIFF=0.12f;
float BOUNCE=0.5f;
float MAX_MOVE=6.0f;
float PARTICLE_RADIUS=5.0f;
int ITERATIONS=5;
float THROW_STRENGTH=3.0f;

// ---------------- Physics ----------------
void ApplyForce(Particle& p,const Vec2& f){ if(!p.pinned) p.acceleration+=f; }

void Integrate(Particle& p,float dt){
    if(p.pinned) return;

    Vec2 vel = p.position - p.oldPosition;
    if(vel.Length() > MAX_MOVE) vel = vel.Normalized() * MAX_MOVE;
    vel *= 0.95f; // damping

    Vec2 temp = p.position;
    p.position += vel + p.acceleration*dt*dt;
    p.oldPosition = temp;
    p.acceleration = Vec2(0,0);
}

void SolveStick(Stick& s){
    Vec2 delta = s.p2->position - s.p1->position;
    float dist = delta.Length();
    if(dist==0) return;
    Vec2 offset = delta*0.5f*(dist-s.restLength)/dist*DIST_STIFF;
    if(!s.p1->pinned) s.p1->position+=offset;
    if(!s.p2->pinned) s.p2->position-=offset;
}

void SolveFloor(Particle& p,float floorY){
    if(p.position.y>floorY){
        float velY=p.position.y - p.oldPosition.y;
        p.position.y=floorY;
        p.oldPosition.y = p.position.y + velY*BOUNCE;
    }
}

void SolveCeiling(Particle& p,float ceilingY){
    if(p.position.y<ceilingY){
        float velY=p.oldPosition.y - p.position.y;
        p.position.y=ceilingY;
        p.oldPosition.y = p.position.y + velY*BOUNCE;
    }
}

void SolveWalls(Particle& p,float leftX,float rightX){
    if(p.position.x<leftX){
        float velX=p.oldPosition.x - p.position.x;
        p.position.x=leftX;
        p.oldPosition.x = p.position.x + velX*BOUNCE;
    }
    if(p.position.x>rightX){
        float velX=p.oldPosition.x - p.position.x;
        p.position.x=rightX;
        p.oldPosition.x = p.position.x + velX*BOUNCE;
    }
}

void SolveParticleCollision(Particle& a,Particle& b,float radius){
    Vec2 delta = b.position - a.position;
    float dist = delta.Length();
    if(dist==0) return;
    if(dist<radius*2){
        Vec2 offset = delta.Normalized()*((radius*2 - dist)/2.0f);
        if(!a.pinned) a.position-=offset;
        if(!b.pinned) b.position+=offset;
    }
}

void SolveShapeMatching(Quad& q){
    Vec2 center = (q.p0->position + q.p1->position + q.p2->position + q.p3->position)/4.0f;
    Vec2 target[4] = {
        center + q.offset0,
        center + q.offset1,
        center + q.offset2,
        center + q.offset3
    };
    Vec2 moves[4] = { (target[0]-q.p0->position)*SHAPE_STIFF,
                       (target[1]-q.p1->position)*SHAPE_STIFF,
                       (target[2]-q.p2->position)*SHAPE_STIFF,
                       (target[3]-q.p3->position)*SHAPE_STIFF };
    for(int i=0;i<4;i++)
        if(moves[i].Length()>MAX_MOVE) moves[i]=moves[i].Normalized()*MAX_MOVE;
    if(!q.p0->pinned) q.p0->position+=moves[0];
    if(!q.p1->pinned) q.p1->position+=moves[1];
    if(!q.p2->pinned) q.p2->position+=moves[2];
    if(!q.p3->pinned) q.p3->position+=moves[3];
}

// ---------------- Softbody ----------------
struct SoftBody {
    std::vector<Particle> particles;
    std::vector<Stick> sticks;
    std::vector<Quad> quads;
    int cols, rows;
    float spacing;

    SoftBody(float startX,float startY,int c,int r,float s){
        cols=c; rows=r; spacing=s;
        float jitter=2.0f;
        for(int y=0;y<rows;y++){
            for(int x=0;x<cols;x++){
                particles.push_back(Particle(Vec2(
                    startX + x*spacing + ((rand()%100)/100.0f)*jitter,
                    startY + y*spacing + ((rand()%100)/100.0f)*jitter
                )));
            }
        }
        for(int y=0;y<rows;y++){
            for(int x=0;x<cols;x++){
                int idx=y*cols+x;
                if(x<cols-1) sticks.push_back(Stick(&particles[idx],&particles[idx+1],spacing));
                if(y<rows-1) sticks.push_back(Stick(&particles[idx],&particles[idx+cols],spacing));
                if(x<cols-1 && y<rows-1){
                    float diag=spacing*1.4142f;
                    sticks.push_back(Stick(&particles[idx],&particles[idx+cols+1],diag));
                    sticks.push_back(Stick(&particles[idx+1],&particles[idx+cols],diag));
                }
            }
        }
        for(int y=0;y<rows-1;y++){
            for(int x=0;x<cols-1;x++){
                int idx=y*cols+x;
                Quad q;
                q.p0=&particles[idx]; q.p1=&particles[idx+1];
                q.p2=&particles[idx+cols]; q.p3=&particles[idx+cols+1];
                Vec2 center=(q.p0->position+q.p1->position+q.p2->position+q.p3->position)/4.0f;
                q.offset0=q.p0->position-center;
                q.offset1=q.p1->position-center;
                q.offset2=q.p2->position-center;
                q.offset3=q.p3->position-center;
                quads.push_back(q);
            }
        }
    }

    Vec2 GetCenter(){
        Vec2 c(0,0);
        for(auto& p:particles) c+=p.position;
        return c/particles.size();
    }
};

// ---------------- Cube-Cube Collision ----------------
void SolveCubeCollision(SoftBody& a, SoftBody& b){
    float ax0 = a.particles[0].position.x;
    float ay0 = a.particles[0].position.y;
    float ax1 = a.particles[a.cols-1].position.x;
    float ay1 = a.particles[(a.rows-1)*a.cols].position.y;

    float bx0 = b.particles[0].position.x;
    float by0 = b.particles[0].position.y;
    float bx1 = b.particles[b.cols-1].position.x;
    float by1 = b.particles[(b.rows-1)*b.cols].position.y;

    if(ax1<bx0 || ax0>bx1 || ay1<by0 || ay0>by1) return;

    float px = std::min(ax1-bx0, bx1-ax0);
    float py = std::min(ay1-by0, by1-ay0);

    if(px < py){
        float push=px/2.0f;
        for(auto& p:a.particles) p.position.x-=push;
        for(auto& p:b.particles) p.position.x+=push;
    }else{
        float push=py/2.0f;
        for(auto& p:a.particles) p.position.y-=push;
        for(auto& p:b.particles) p.position.y+=push;
    }
}

// ---------------- Slider ----------------
float Slider(Rectangle rect,float value,float minVal,float maxVal){
    DrawRectangleRec(rect,LIGHTGRAY);
    float t = (value-minVal)/(maxVal-minVal);
    Rectangle knob = {rect.x + t*rect.width-5,rect.y-5,10,rect.height+10};
    DrawRectangleRec(knob,DARKGRAY);

    Vector2 mp = GetMousePosition();
    if(IsMouseButtonDown(MOUSE_LEFT_BUTTON) &&
       mp.x >= rect.x && mp.x <= rect.x+rect.width &&
       mp.y >= rect.y-10 && mp.y <= rect.y+rect.height+10){
           float nt = (mp.x - rect.x)/rect.width;
           nt = nt<0?0:(nt>1?1:nt);
           value = minVal + nt*(maxVal-minVal);
    }
    return value;
}

// ---------------- Main ----------------
int main(){
    const int screenWidth=1800,screenHeight=1000;
    InitWindow(screenWidth,screenHeight,"Mai Boi");
    SetTargetFPS(60);

    float dt=1.0f/60.0f;
    Vec2 gravity(0,2000.0f);

    std::vector<SoftBody*> cubes;
    cubes.push_back(new SoftBody(300,300,10,10,30));
    cubes.push_back(new SoftBody(900,300,10,10,30));

    Particle* grabbedParticle=nullptr;
    Vec2 prevMousePos(0,0);

    while(!WindowShouldClose()){
        Vector2 mp = GetMousePosition();
        Vec2 mousePos(mp.x, mp.y);
        Vec2 mouseVel = (mousePos - prevMousePos) * THROW_STRENGTH;

        // ---------------- Mouse Drag / Flick ----------------
        if(IsMouseButtonPressed(MOUSE_LEFT_BUTTON)){
            float minDist=std::numeric_limits<float>::max();
            grabbedParticle=nullptr;
            for(auto cube:cubes)
                for(auto& p:cube->particles){
                    float d=(p.position.x-mousePos.x)*(p.position.x-mousePos.x)+
                            (p.position.y-mousePos.y)*(p.position.y-mousePos.y);
                    if(d<minDist && d<400){ minDist=d; grabbedParticle=&p; }
                }
        }

        if(IsMouseButtonDown(MOUSE_LEFT_BUTTON) && grabbedParticle){
            SoftBody* cubeGrab=nullptr;
            for(auto cube:cubes)
                for(auto& p:cube->particles)
                    if(&p==grabbedParticle) cubeGrab=cube;
            if(cubeGrab){
                for(auto& p:cubeGrab->particles){
                    p.oldPosition = p.position - mouseVel; // flick velocity
                }
            }
            grabbedParticle->position = mousePos;
        }

        if(IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) grabbedParticle=nullptr;

        prevMousePos = mousePos;

        // ---------------- Physics ----------------
        for(auto cube:cubes){
            for(auto& p:cube->particles) ApplyForce(p,gravity);
            for(auto& p:cube->particles) Integrate(p,dt);

            for(int it=0;it<ITERATIONS;it++){
                for(auto& s:cube->sticks) SolveStick(s);
                for(auto& p:cube->particles){
                    SolveFloor(p,screenHeight-50);
                    SolveCeiling(p,50);
                    SolveWalls(p,50,screenWidth-50);
                }
                for(auto& q:cube->quads) SolveShapeMatching(q);
                for(size_t j=0;j<cube->particles.size();j++)
                    for(size_t k=j+1;k<cube->particles.size();k++)
                        SolveParticleCollision(cube->particles[j],cube->particles[k],PARTICLE_RADIUS);
            }
        }

        for(size_t i=0;i<cubes.size();i++)
            for(size_t j=i+1;j<cubes.size();j++)
                SolveCubeCollision(*cubes[i],*cubes[j]);

        // ---------------- Add / Delete cubes ----------------
        if(IsKeyPressed(KEY_A)){
            cubes.push_back(new SoftBody(mousePos.x, mousePos.y,10,10,30));
        }
        if(IsKeyPressed(KEY_D)){
            for(auto it=cubes.begin(); it!=cubes.end(); ++it){
                SoftBody* cube=*it;
                Vec2 center=cube->GetCenter();
                if((mousePos-center).Length()<100){
                    delete cube;
                    cubes.erase(it);
                    break;
                }
            }
        }

        // ---------------- Draw ----------------
        BeginDrawing();
        ClearBackground(RAYWHITE);

        DrawRectangle(0,0,screenWidth,50,GRAY);
        DrawRectangle(0,screenHeight-50,screenWidth,50,GRAY);
        DrawRectangle(0,0,50,screenHeight,GRAY);
        DrawRectangle(screenWidth-50,0,50,screenHeight,GRAY);

        for(auto cube:cubes){
            float left=cube->particles[0].position.x;
            float top=cube->particles[0].position.y;
            float right=cube->particles[cube->cols-1].position.x;
            float bottom=cube->particles[(cube->rows-1)*cube->cols].position.y;
            DrawRectangle((int)left,(int)top,(int)(right-left),(int)(bottom-top),Fade(SKYBLUE,0.4f));

            for(auto& q:cube->quads){
                Vector2 v0={q.p0->position.x,q.p0->position.y};
                Vector2 v1={q.p1->position.x,q.p1->position.y};
                Vector2 v2={q.p3->position.x,q.p3->position.y};
                Vector2 v3={q.p2->position.x,q.p2->position.y};
                DrawTriangle(v0,v1,v2, Fade(SKYBLUE,0.7f));
                DrawTriangle(v0,v2,v3, Fade(SKYBLUE,0.7f));
            }
        }

        // ---------------- Sliders ----------------
        DIST_STIFF = Slider({50, 60, 200, 20}, DIST_STIFF, 0.0f, 1.0f);
        SHAPE_STIFF = Slider({50, 90, 200, 20}, SHAPE_STIFF, 0.0f, 1.0f);
        gravity.y = Slider({50, 120, 200, 20}, gravity.y, 0.0f, 4000.0f);
        PARTICLE_RADIUS = Slider({50, 150, 200, 20}, PARTICLE_RADIUS, 1.0f, 15.0f);
        THROW_STRENGTH = Slider({50,180,200,20}, THROW_STRENGTH,1.0f,10.0f);

        DrawText("COMP SQUISHY CUBES",50,20,20,DARKGRAY);
        DrawText(TextFormat("Dist Stiff: %.2f",DIST_STIFF),260,60,15,BLACK);
        DrawText(TextFormat("Shape Stiff: %.2f",SHAPE_STIFF),260,90,15,BLACK);
        DrawText(TextFormat("Gravity: %.0f",gravity.y),260,120,15,BLACK);
        DrawText(TextFormat("Particle Radius: %.1f",PARTICLE_RADIUS),260,150,15,BLACK);
        DrawText(TextFormat("Throw Strength: %.2f",THROW_STRENGTH),260,180,15,BLACK);
        DrawText("Press A to add cube, D to delete cube under mouse",50,200,15,BLACK);

        EndDrawing();
    }

    // Cleanup
    for(auto cube:cubes) delete cube;
    CloseWindow();
    return 0;
}
