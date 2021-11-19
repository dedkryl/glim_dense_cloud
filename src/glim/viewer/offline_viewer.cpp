#include <glim/viewer/offline_viewer.hpp>

#include <glim/util/console_colors.hpp>

#include <portable-file-dialogs.h>
#include <glk/io/ply_io.hpp>
#include <guik/recent_files.hpp>
#include <guik/progress_modal.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace glim {

OfflineViewer::OfflineViewer() {}

OfflineViewer::~OfflineViewer() {}

void OfflineViewer::setup_ui() {
  auto viewer = guik::LightViewer::instance();
  viewer->register_ui_callback("main_menu", [this] { main_menu(); });

  progress_modal.reset(new guik::ProgressModal("offline_viewer_progress"));
}

void OfflineViewer::main_menu() {
  bool start_open_map = false;
  bool start_close_map = false;
  bool start_save_map = false;
  bool start_export_map = false;

  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      if (ImGui::MenuItem("Open Map")) {
        start_open_map = true;
      }

      if (ImGui::MenuItem("Close Map")) {
        if (pfd::message("Warning", "Close the map?").result() == pfd::button::ok) {
          start_close_map = true;
        }
      }

      if (ImGui::BeginMenu("Save")) {
        if (ImGui::MenuItem("Save Map")) {
          start_save_map = true;
        }

        if (ImGui::MenuItem("Export Points")) {
          start_export_map = true;
        }

        ImGui::EndMenu();
      }

      if (ImGui::MenuItem("Quit")) {
        if (pfd::message("Warning", "Quit?").result() == pfd::button::ok) {
          request_to_terminate = true;
        }
      }

      ImGui::EndMenu();
    }

    ImGui::EndMainMenuBar();
  }

  // open map
  if (start_open_map) {
    guik::RecentFiles recent_files("offline_viewer_open");
    const std::string path = pfd::select_folder("Select a dump directory", recent_files.most_recent()).result();
    if (!path.empty()) {
      recent_files.push(path);
      progress_modal->open<std::shared_ptr<GlobalMapping>>("open", [this, path](guik::ProgressInterface& progress) { return load_map(progress, path); });
    }
  }
  auto open_result = progress_modal->run<std::shared_ptr<GlobalMapping>>("open");
  if (open_result) {
    if (!(*open_result)) {
      pfd::message("Error", "Failed to load map").result();
    } else {
      async_global_mapping.reset(new glim::AsyncGlobalMapping(*open_result));
    }
  }

  // save map
  if (start_save_map) {
    guik::RecentFiles recent_files("offline_viewer_save");
    const std::string path = pfd::select_folder("Select a directory to save the map", recent_files.most_recent()).result();
    if (!path.empty()) {
      recent_files.push(path);
      progress_modal->open<bool>("save", [this, path](guik::ProgressInterface& progress) { return save_map(progress, path); });
    }
  }
  auto save_result = progress_modal->run<bool>("save");

  // export map
  if (start_export_map) {
    guik::RecentFiles recent_files("offline_viewer_export");
    const std::string path = pfd::save_file("Select the file destination", recent_files.most_recent(), {"PLY", "*.ply"}).result();
    if (!path.empty()) {
      recent_files.push(path);
      progress_modal->open<bool>("export", [this, path](guik::ProgressInterface& progress) { return export_map(progress, path); });
    }
  }
  auto export_result = progress_modal->run<bool>("export");

  // close map
  if (start_close_map) {
    async_global_mapping->join();
    async_global_mapping.reset();
    clear();
  }
}

std::shared_ptr<GlobalMapping> OfflineViewer::load_map(guik::ProgressInterface& progress, const std::string& path) {
  progress.set_title("Load map");
  progress.set_text("Now loading");
  progress.set_maximum(1);

  std::shared_ptr<glim::GlobalMapping> global_mapping(new glim::GlobalMapping);
  if (!global_mapping->load(path)) {
    std::cerr << console::bold_red << "error: failed to load " << path << console::reset << std::endl;
    return nullptr;
  }

  return global_mapping;
}

bool OfflineViewer::save_map(guik::ProgressInterface& progress, const std::string& path) {
  progress.set_title("Save map");
  progress.set_text("Now saving");
  async_global_mapping->save(path);
  return true;
}

bool OfflineViewer::export_map(guik::ProgressInterface& progress, const std::string& path) {
  progress.set_title("Export points");
  progress.set_text("Concatenating submaps");
  progress.set_maximum(3);
  progress.increment();
  const auto points = async_global_mapping->export_points();

  progress.set_text("Writing to file");
  progress.increment();
  glk::save_ply_binary(path, points.data(), points.size());

  return true;
}

}  // namespace glim