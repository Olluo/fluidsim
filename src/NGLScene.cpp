#include <QGuiApplication>
#include <QMouseEvent>

#include "NGLScene.h"
#include <chrono>
#include <fmt/format.h>
#include <ngl/NGLInit.h>
#include <ngl/Random.h>
#include <ngl/ShaderLib.h>
#include <ngl/SimpleVAO.h>
#include <ngl/Transformation.h>
#include <ngl/VAOFactory.h>
#include <ngl/Vec2.h>

constexpr size_t c_sampleSize = 500;

NGLScene::NGLScene()
{
  setTitle("2D Grid-Based Fluid Simulation");

  // we need to add an initial value to the rolling average to make code simpler
  m_updateTime.push_back(1);
}

NGLScene::~NGLScene()
{
  std::cout << "Shutting down NGL, removing VAO's and Shaders\n";
}

void NGLScene::resizeGL(int _w, int _h)
{
  m_project = ngl::perspective(45.0f, static_cast<float>(_w) / _h, 0.01f, 150.0f);
  m_win.width = static_cast<int>(_w * devicePixelRatio());
  m_win.height = static_cast<int>(_h * devicePixelRatio());
  m_text->setScreenSize(_w, _h);
}

void NGLScene::initializeGL()
{
  // we need to initialise the NGL lib which will load all of the OpenGL functions, this must
  // be done once we have a valid GL context but before we call any GL commands. If we dont do
  // this everything will crash
  ngl::NGLInit::initialize();

  glClearColor(0.4f, 0.4f, 0.4f, 1.0f); // Grey Background
  // enable depth testing for drawing

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_MULTISAMPLE);
  glViewport(0, 0, width(), height());

  // Now we will create a basic Camera from the graphics library
  // This is a static camera so it only needs to be set once
  // First create Values for the camera position
  ngl::Vec3 from(static_cast<float>(c_size - 1) / 2.0f, c_size + 17, static_cast<float>(c_size - 1) / 2.0f);
  ngl::Vec3 to(static_cast<float>(c_size - 1) / 2.0f, 0, static_cast<float>(c_size - 1) / 2.0f);
  ngl::Vec3 up(0, 0, 1);
  m_view = ngl::lookAt(from, to, up);

  // set the shape using FOV 45 Aspect Ratio based on Width and Height
  m_project = ngl::perspective(45, 1024.0f / 720.0f, 0.01f, 150.0f);

  // now to load the shader and set the values
  ngl::ShaderLib::loadShader("PosDir", "shaders/PosDirVertex.glsl", "shaders/PosDirFragment.glsl", "shaders/PosDirGeo.glsl");
  ngl::ShaderLib::use("PosDir");

  // Create the fluid with viscosity 20.0f and a time step of 0.0000001f
  m_fluidGrid = std::make_unique<FluidGrid>(20.0f, 0.0000001f);

  m_text = std::make_unique<ngl::Text>("fonts/Arial.ttf", 18);
  m_text->setColour(1.0f, 1.0f, 0.0f);

  startTimer(20);
}

void NGLScene::paintGL()
{
  // clear the screen and depth buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  m_mouseGlobalTX.m_m[3][1] = m_modelPos.m_z;

  ngl::ShaderLib::use("PosDir");
  ngl::Mat4 MVP;
  MVP = m_project * m_view * m_mouseGlobalTX;
  ngl::ShaderLib::setUniform("MVP", MVP);

  glPointSize(100);

  auto drawbegin = std::chrono::steady_clock::now();
  m_fluidGrid->draw();
  auto drawend = std::chrono::steady_clock::now();
  auto updateTime = std::accumulate(std::begin(m_updateTime), std::end(m_updateTime), 0) / m_updateTime.size();

  m_text->renderText(10, 50, "[Spacebar] to reset");
  m_text->renderText(10, 30, fmt::format("- Draw took {0} uS", std::chrono::duration_cast<std::chrono::microseconds>(drawend - drawbegin).count()));
  m_text->renderText(10, 10, fmt::format("- Update took {0} uS for {1} particles", updateTime, m_fluidGrid->getNumParticles()));
}

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::mouseMoveEvent(QMouseEvent *_event)
{
}

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::mousePressEvent(QMouseEvent *_event)
{
  // that method is called when the mouse button is pressed in this case we
  // store the value where the maouse was clicked (x,y) and set the Rotate flag to true
  if (_event->button() == Qt::LeftButton)
  {
    m_win.x0 = _event->x();
    m_win.y0 = _event->y();
  }
}

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::mouseReleaseEvent(QMouseEvent *_event)
{
  // this event is called when the mouse button is released and we use this to add velocity to the fluid
  if (_event->button() == Qt::LeftButton)
  {
    int x1 = _event->x();
    int y1 = _event->y();

    ngl::Vec2 velocity{static_cast<ngl::Real>(m_win.x0 - x1), static_cast<ngl::Real>(m_win.y0 - y1)};

    // Only normalize if the length isn't zero (divide by zero error)
    if (velocity.lengthSquared() != 0)
    {
      velocity.normalize();
    }
    velocity *= m_win.scale;

    // Convert mouse coordinates to grid coordinates
    int x = c_size - static_cast<int>(static_cast<float>(m_win.x0) / m_win.width * c_size);
    int y = c_size - static_cast<int>(static_cast<float>(m_win.y0) / m_win.height * c_size);

    // add velocity in a 3x3 area where the mouse clicked with direction of the drag
    m_fluidGrid->addVelocity(ngl::Vec2{static_cast<ngl::Real>(x - 1), static_cast<ngl::Real>(y - 1)}, velocity);
    m_fluidGrid->addVelocity(ngl::Vec2{static_cast<ngl::Real>(x), static_cast<ngl::Real>(y - 1)}, velocity);
    m_fluidGrid->addVelocity(ngl::Vec2{static_cast<ngl::Real>(x - 1), static_cast<ngl::Real>(y - 1)}, velocity);

    m_fluidGrid->addVelocity(ngl::Vec2{static_cast<ngl::Real>(x - 1), static_cast<ngl::Real>(y)}, velocity);
    m_fluidGrid->addVelocity(ngl::Vec2{static_cast<ngl::Real>(x), static_cast<ngl::Real>(y)}, velocity);
    m_fluidGrid->addVelocity(ngl::Vec2{static_cast<ngl::Real>(x - 1), static_cast<ngl::Real>(y)}, velocity);

    m_fluidGrid->addVelocity(ngl::Vec2{static_cast<ngl::Real>(x - 1), static_cast<ngl::Real>(y + 1)}, velocity);
    m_fluidGrid->addVelocity(ngl::Vec2{static_cast<ngl::Real>(x), static_cast<ngl::Real>(y + 1)}, velocity);
    m_fluidGrid->addVelocity(ngl::Vec2{static_cast<ngl::Real>(x - 1), static_cast<ngl::Real>(y + 1)}, velocity);

    update();
  }
}

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::wheelEvent(QWheelEvent *_event)
{

  // check the diff of the wheel position (0 means no change)
  if (_event->angleDelta().y() > 0)
  {
    m_modelPos.m_z += ZOOM;
  }
  else if (_event->angleDelta().y() < 0)
  {
    m_modelPos.m_z -= ZOOM;
  }
  m_modelPos.m_z = std::clamp(m_modelPos.m_z, 0.0f, static_cast<ngl::Real>(c_size + 17));
  update();
}
//----------------------------------------------------------------------------------------------------------------------

void NGLScene::keyPressEvent(QKeyEvent *_event)
{
  // this method is called every time the main window receives a key event.
  // we then switch on the key value and set the camera in the GLWindow
  switch (_event->key())
  {
  case Qt::Key_Escape:
    QGuiApplication::exit(EXIT_SUCCESS);
    break;
  case Qt::Key_Space:
    m_fluidGrid->reset();
    break;
  default:
    break;
  }

  update();
}

void NGLScene::timerEvent(QTimerEvent *)
{
  auto updatebegin = std::chrono::steady_clock::now();
  m_fluidGrid->step();
  auto updateend = std::chrono::steady_clock::now();

  // add to the rolling average
  m_updateTime.push_back(std::chrono::duration_cast<std::chrono::microseconds>(updateend - updatebegin).count());

  // if greater than sample size remove front element
  if (m_updateTime.size() > c_sampleSize)
  {
    m_updateTime.pop_front();
  }

  update();
}
