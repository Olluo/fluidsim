#ifndef WINDOWPARAMS_H_
#define WINDOWPARAMS_H_

struct WinParams
{
  // window width
  int width = 800;
  // window height
  int height = 800;
  int x0 = 0;
  int y0 = 0;
  float scale = 5.0f;
};

//----------------------------------------------------------------------------------------------------------------------
/// @brief the increment for x/y translation with mouse movement
//----------------------------------------------------------------------------------------------------------------------
constexpr float INCREMENT = 0.01f;
//----------------------------------------------------------------------------------------------------------------------
/// @brief the increment for the wheel zoom
//----------------------------------------------------------------------------------------------------------------------
constexpr float ZOOM = 5.0f;

#endif
