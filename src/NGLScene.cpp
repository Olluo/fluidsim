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

NGLScene::NGLScene(uint32_t _size)
{
  setTitle("Simple Grid Particle System SOA");
  m_gridSize = _size;
  // we need to add an initial value to the rolling average to make code simpler
  m_updateTime.push_back(1);
}

NGLScene::~NGLScene()
{
  std::cout << "Shutting down NGL, removing VAO's and Shaders\n";
}

void NGLScene::resizeGL(int _w, int _h)
{
  m_project = ngl::perspective(45.0f, static_cast<float>(_w) / _h, 0.05f, 350.0f);
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
  // Now we will create a basic Camera from the graphics library
  // This is a static camera so it only needs to be set once
  // First create Values for the camera position
  ngl::Vec3 from(static_cast<float>(m_gridSize - 1) / 2.0f, m_gridSize + 26, static_cast<float>(m_gridSize - 1) / 2.0f);
  ngl::Vec3 to(static_cast<float>(m_gridSize - 1) / 2.0f, 0, static_cast<float>(m_gridSize - 1) / 2.0f);
  ngl::Vec3 up(0, 0, 1);

  m_view = ngl::lookAt(from, to, up);
  // set the shape using FOV 45 Aspect Ratio based on Width and Height
  // The final two are near and far clipping planes of 0.5 and 10
  m_project = ngl::perspective(45, 1024.0f / 720.0f, 0.01f, 150);

  // now to load the shader and set the values
  // grab an instance of shader manager
  ngl::ShaderLib::loadShader("PosDir", "shaders/PosDirVertex.glsl", "shaders/PosDirFragment.glsl", "shaders/PosDirGeo.glsl");
  // ngl::ShaderLib::use("nglColourShader");
  ngl::ShaderLib::use("PosDir");

  ngl::ShaderLib::setUniform("Colour", 1.0f, 1.0f, 1.0f, 1.0f);
  glViewport(0, 0, width(), height());
  m_fluid = std::make_unique<Fluid>(m_gridSize, 0.2f, 0.0f, 0.0000001f);
  m_fluid->step();
  // m_fluid->addVelocity(m_gridSize / 2.0f, m_gridSize / 2.0f, 5, 0);
  ngl::VAOPrimitives::createLineGrid("lineGrid", m_gridSize, m_gridSize, 1);
  ngl::ShaderLib::use(ngl::nglColourShader);
  ngl::ShaderLib::setUniform("Colour", 1.0f, 1.0f, 1.0f, 1.0f);
  m_text = std::make_unique<ngl::Text>("fonts/Arial.ttf", 18);
  m_text->setColour(1.0f, 1.0f, 0.0f);

  startTimer(20);
}

void NGLScene::paintGL()
{
  // clear the screen and depth buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  //  m_mouseGlobalTX.m_m[3][0] = m_modelPos.m_x;
  m_mouseGlobalTX.m_m[3][1] = m_modelPos.m_z;
  //  m_mouseGlobalTX.m_m[3][2] = m_modelPos.m_z;
  ngl::ShaderLib::use("PosDir");
  // ngl::ShaderLib::use("nglColourShader");
  ngl::Mat4 MVP;
  MVP = m_project * m_view * m_mouseGlobalTX;

  ngl::ShaderLib::setUniform("MVP", MVP);
  glPointSize(100);

  auto drawbegin = std::chrono::steady_clock::now();

  m_fluid->step();
  m_fluid->draw();
  auto drawend = std::chrono::steady_clock::now();

  std::string text = fmt::format("Draw took {0} uS", std::chrono::duration_cast<std::chrono::microseconds>(drawend - drawbegin).count());
  m_text->renderText(10, 30, text);

  auto updateTime = std::accumulate(std::begin(m_updateTime), std::end(m_updateTime), 0) / m_updateTime.size();
  text = fmt::format("Update took {0} uS for {1} particles", updateTime, m_fluid->getNumParticles());
  m_text->renderText(10, 10, text);

  ngl::ShaderLib::use(ngl::nglColourShader);
  ngl::ShaderLib::setUniform("MVP", MVP);
  // ngl::VAOPrimitives::draw("lineGrid");
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
  // that event is called when the mouse button is released
  // we then set Rotate to false
  if (_event->button() == Qt::LeftButton)
  {
    int x1 = _event->x();
    int y1 = _event->y();

    float dx = m_win.scale * (x1 - m_win.x0);
    float dy = m_win.scale * (y1 - m_win.y0);

    int x = m_gridSize - static_cast<int>(static_cast<float>(m_win.x0) / m_win.width * m_gridSize);
    int y = m_gridSize - static_cast<int>(static_cast<float>(m_win.y0) / m_win.height * m_gridSize);

    std::cout << "pos 0  = (" << m_win.x0 << ", " << m_win.y0 << ")\n";
    std::cout << "pos 1  = (" << x1 << ", " << y1 << ")\n";
    std::cout << "delXY  = (" << dx << ", " << dy << ")\n";
    std::cout << "gridXY = (" << x << ", " << y << ")\n";
    std::cout << "===================\n";

    m_fluid->addVelocity(ngl::Vec2{static_cast<ngl::Real>(x), static_cast<ngl::Real>(y)}, ngl::Vec2{dx, dy});
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
  update();
}
//----------------------------------------------------------------------------------------------------------------------

void NGLScene::keyPressEvent(QKeyEvent *_event)
{
  // this method is called every time the main window recives a key event.
  // we then switch on the key value and set the camera in the GLWindow
  switch (_event->key())
  {
  // escape key to quite
  case Qt::Key_Escape:
    QGuiApplication::exit(EXIT_SUCCESS);
    break;
  // turn on wirframe rendering
  case Qt::Key_W:
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    break;
  // turn off wire frame
  case Qt::Key_S:
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    break;
  // show full screen
  case Qt::Key_F:
    showFullScreen();
    break;
  // show windowed
  case Qt::Key_N:
    showNormal();
    break;

  case Qt::Key_1:
    m_fluid->toggleDrawMode(Fluid::DrawMode::SINGLEBUFFER);
    break;
  case Qt::Key_2:
    m_fluid->toggleDrawMode(Fluid::DrawMode::MULTIBUFFER);
    break;
  case Qt::Key_Space:
    m_fluid->reset();
    break;

  default:
    break;
  }
  // finally update the GLWindow and re-draw
  //if (isExposed())
  update();
}

void NGLScene::timerEvent(QTimerEvent *)
{
  auto updatebegin = std::chrono::steady_clock::now();
  // m_fluid->update(0.01f);
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
