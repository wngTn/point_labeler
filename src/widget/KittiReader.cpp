#include <stdint.h>
#include <widget/KittiReader.h>

#include <QtCore/QDir>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <clocale>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "happly.h"
#include "rv/string_utils.h"

void KittiReader::initialize(const QString& directory) {
  std::setlocale(LC_ALL, "en_US.utf8");
  pointclouds_filenames_.clear();
  label_filenames_.clear();
  image_filenames_.clear();
  poses_.clear();

  pointsCache_.clear();
  labelCache_.clear();
  tiles_.clear();

  base_dir_ = QDir(directory);
  QDir pointclouds_dir(base_dir_.filePath("pointclouds"));
  QStringList entries = pointclouds_dir.entryList(QDir::Files, QDir::Name);
  for (int32_t i = 0; i < entries.size(); ++i) {
    pointclouds_filenames_.push_back(pointclouds_dir.filePath(entries.at(i)).toStdString());
  }

  if (!base_dir_.exists("calib.txt")) {
    calib_.initializeDefault();
    // throw std::runtime_error("Missing calibration file: " + base_dir_.filePath("calib.txt").toStdString());
  } else {
    calib_.initialize(base_dir_.filePath("calib.txt").toStdString());
  }

  if (!base_dir_.exists("poses.txt")) {
    defaultPoses = true;
    // throw std::runtime_error("Missing calibration file: " + base_dir_.filePath("calib.txt").toStdString());
  } else {
    readPoses(base_dir_.filePath("poses.txt").toStdString(), poses_);
  }

  // create label dir, etc.
  QDir labels_dir(base_dir_.filePath("labels"));

  // find corresponding label files.
  if (!labels_dir.exists()) base_dir_.mkdir("labels");

  for (uint32_t i = 0; i < pointclouds_filenames_.size(); ++i) {
    QString filename = QFileInfo(QString::fromStdString(pointclouds_filenames_[i])).baseName() + ".label";
    if (!labels_dir.exists(filename)) {
      std::ifstream in(pointclouds_filenames_[i].c_str(), std::ifstream::in);
      // TODO falsch!
      uint32_t num_points = 0;
      std::string currentLine;
      bool headerFound = false;
      while (std::getline(in, currentLine)) {
        if (currentLine.compare("end_header") == 0) {
          headerFound = true;
          break;
        }
      }
      if (headerFound) {
        // while (std::getline(in, currentLine)) {
        //   num_points++;
        // }
        // TODO maybe not the best way, but since we only do it once it's okay I guess 
        happly::PLYData plyIn(pointclouds_filenames_[i].c_str(), false);
        plyIn.validate();

        num_points = plyIn.getVertexPositions().size();
        if (num_points > 1) {
          // num_points--;
          std::cout << num_points << " points, creating labels" << std::endl;
          std::ofstream out(labels_dir.filePath(filename).toStdString().c_str());

          std::vector<uint32_t> labels(num_points, 0);
          out.write(reinterpret_cast<const char*>(labels.data()), num_points * sizeof(uint32_t));

          out.close();
        } else {
          std::cout << "No points found!" << std::endl;
        }
      } else {
        std::cout << "Could not find header..." << std::endl;
      }
      in.close();
    }

    label_filenames_.push_back(labels_dir.filePath(filename).toStdString());
  }

  std::string missing_img = QDir::currentPath().toStdString() + "/../assets/missing.png";
  QDir image_dir(base_dir_.filePath("image_2"));
  for (uint32_t i = 0; i < pointclouds_filenames_.size(); ++i) {
    QString filename = QFileInfo(QString::fromStdString(pointclouds_filenames_[i])).baseName() + ".png";
    if (image_dir.exists(filename)) {
      image_filenames_.push_back(image_dir.filePath(filename).toStdString());
    } else {
      std::cout << "Note: Missing " << pointclouds_filenames_[i] << ".png" << std::endl;
      image_filenames_.push_back(missing_img);
    }
  }

  // assumes that (0,0,0) is always the start.
  Eigen::Vector2f min = Eigen::Vector2f::Zero();
  Eigen::Vector2f max = Eigen::Vector2f::Zero();

  if (defaultPoses) {
    min.x() = -maxDistance_;
    min.y() = -maxDistance_;
    max.x() = maxDistance_;
    max.y() = maxDistance_;
  } else
    for (uint32_t i = 0; i < poses_.size(); ++i) {
      Eigen::Vector4f t = poses_[i].col(3);

      min.x() = std::min(t.x() - maxDistance_, min.x());
      min.y() = std::min(t.y() - maxDistance_, min.y());
      max.x() = std::max(t.x() + maxDistance_, max.x());
      max.y() = std::max(t.y() + maxDistance_, max.y());
    }

  //  std::cout << "tileSize = " << tileSize_ << std::endl;
  //  std::cout << "min = " << min << ", max = " << max << std::endl;

  offset_.x() = std::ceil((std::abs(min.x()) - 0.5 * tileSize_) / tileSize_) * tileSize_ + 0.5 * tileSize_;
  offset_.y() = std::ceil((std::abs(min.y()) - 0.5 * tileSize_) / tileSize_) * tileSize_ + 0.5 * tileSize_;

  //  std::cout << "offset = " << offset_ << std::endl;

  numTiles_.x() = std::ceil((std::abs(min.x()) - 0.5 * tileSize_) / tileSize_) +
                  std::ceil((max.x() - 0.5 * tileSize_) / tileSize_) + 1;
  numTiles_.y() = std::ceil((std::abs(min.y()) - 0.5 * tileSize_) / tileSize_) +
                  std::ceil((max.y() - 0.5 * tileSize_) / tileSize_) + 1;

  //  std::cout << "numTiles = " << numTiles_ << std::endl;

  tiles_.clear();
  tiles_.resize(numTiles_.x() * numTiles_.y());

  Eigen::Vector2f idxRadius(maxDistance_ / tileSize_, maxDistance_ / tileSize_);

  for (uint32_t i = 0; i < uint32_t(numTiles_.x()); ++i) {
    for (uint32_t j = 0; j < uint32_t(numTiles_.y()); ++j) {
      auto& tile = tiles_[tileIdxToOffset(i, j)];

      tile.i = i;
      tile.j = j;
      tile.x = i * tileSize_ - offset_.x() + 0.5 * tileSize_;
      tile.y = j * tileSize_ - offset_.y() + 0.5 * tileSize_;
      tile.size = tileSize_;
    }
  }

  trajectory_.clear();

  Eigen::Vector2f e(0.5 * tileSize_, 0.5 * tileSize_);

  uint32_t s;
  Eigen::Vector2f t;
  if (defaultPoses) {
    t = Eigen::Vector2f(0, 0);
    s = count();
  } else {
    s = poses_.size();
  }
  for (uint32_t i = 0; i < s; ++i) {
    if (!defaultPoses) {
      t = poses_[i].col(3).head(2);
    }
    Eigen::Vector2f idx((t.x() + offset_.x()) / tileSize_, (t.y() + offset_.y()) / tileSize_);

    trajectory_.push_back(Eigen::Vector2f((t.x() + offset_.x()) / tileSize_, (t.y() + offset_.y()) / tileSize_));

    //    tiles_[tileIdxToOffset(uint32_t(idx.x()), uint32_t(idx.y()))].indexes.push_back(i);
    //    uint32_t u_min = std::max(int32_t(idx.x() - idxRadius.x()), 0);
    //    uint32_t u_max = std::min(int32_t(std::ceil(idx.x() + idxRadius.x())), numTiles_.x());
    //    uint32_t v_min = std::max(int32_t(idx.y() - idxRadius.y()), 0);
    //    uint32_t v_max = std::min(int32_t(std::ceil(idx.y() + idxRadius.y())), numTiles_.y());

    // FIXME: workaround check all tiles.
    for (uint32_t u = 0; u < uint32_t(numTiles_.x()); ++u) {
      for (uint32_t v = 0; v < uint32_t(numTiles_.y()); ++v) {
        auto& tile = tiles_[tileIdxToOffset(u, v)];
        Eigen::Vector2f q = t - Eigen::Vector2f(tile.x, tile.y);
        q[0] = std::abs(q[0]);
        q[1] = std::abs(q[1]);

        // check for exact overlap (see Behley et al., ICRA, 2015)
        if (std::max(q[0], q[1]) > e[0] + maxDistance_) continue;  // definitely outside.
        if (std::min(q[0], q[1]) < e[0] || (q - e).norm() < maxDistance_) {
          tile.indexes.push_back(i);
        }
      }
    }
  }

  // sanity check:

  for (auto& t : tiles_) {
    std::sort(t.indexes.begin(), t.indexes.end());
    //    std::cout << "Tile has " << t.indexes.size() << " tiles associated." << std::endl;
    for (uint32_t i = 1; i < t.indexes.size(); ++i) {
      if (t.indexes[i - 1] == t.indexes[i]) {
        std::cout << "found duplicate!" << std::endl;
      }
    }
  }

  uint32_t tileCount = 0;
  for (uint32_t i = 0; i < uint32_t(numTiles_.x()); ++i) {
    for (uint32_t j = 0; j < uint32_t(numTiles_.y()); ++j) {
      auto& tile = tiles_[tileIdxToOffset(i, j)];

      std::sort(tile.indexes.begin(), tile.indexes.end());
      if (tile.indexes.size() > 0) tileCount += 1;
    }
  }

  std::cout << "#tiles  = " << tileCount << std::endl;

  // meta information for faster loading.
  if (base_dir_.exists("instances.txt")) {
    std::cout << "Reading instances.txt..." << std::flush;

    std::ifstream in(base_dir_.filePath("instances.txt").toStdString());

    while (in.good()) {
      std::string line;
      std::getline(in, line);
      if (line.size() == 0) break;

      std::vector<std::string> tokens = rv::split(line, ":");
      if (tokens.size() != 2) {
        throw std::runtime_error("Invalid instance meta information found!");
      }

      uint32_t label = boost::lexical_cast<uint32_t>(tokens[0]);
      uint32_t maxInstanceId = boost::lexical_cast<uint32_t>(tokens[1]);
      maxInstanceIds_[label] = maxInstanceId;

      in.peek();
    }

    in.close();
    std::cout << "finished." << std::endl;
  } else {
    std::cout << "Generating intances.txt" << std::flush;
    // get the counts from the label files.
    for (const std::string& filename : label_filenames_) {
      std::vector<uint32_t> labels;
      readLabels(filename, labels);

      for (uint32_t instance_label : labels) {
        uint32_t instanceId = (instance_label >> 16) & uint32_t(0xFFFF);
        uint32_t label = instance_label & uint32_t(0xFFFF);
        if (maxInstanceIds_.find(label) == maxInstanceIds_.end())
          maxInstanceIds_[label] = instanceId;
        else
          maxInstanceIds_[label] = std::max(instanceId, maxInstanceIds_[label]);
      }
    }

    // directly update meta information:
    updateMetaInformation(maxInstanceIds_);
  }
}

void KittiReader::updateMetaInformation(const std::map<uint32_t, uint32_t>& maxInstanceIds) {
  std::ofstream out(base_dir_.filePath("instances.txt").toStdString().c_str());
  for (auto it = maxInstanceIds.begin(); it != maxInstanceIds.end(); ++it) {
    out << it->first << ":" << it->second << std::endl;
  }
  out.close();
}

void KittiReader::retrieve(const Eigen::Vector3f& position, std::vector<uint32_t>& indexes,
                           std::vector<PointcloudPtr>& points, std::vector<LabelsPtr>& labels,
                           std::vector<std::string>& images) {
  Eigen::Vector2f idx((position.x() + offset_.x()) / tileSize_, (position.y() + offset_.y()) / tileSize_);

  retrieve(idx.x(), idx.y(), indexes, points, labels, images);
}

void KittiReader::retrieve(uint32_t i, uint32_t j, std::vector<uint32_t>& indexes, std::vector<PointcloudPtr>& points,
                           std::vector<LabelsPtr>& labels, std::vector<std::string>& images) {
  indexes.clear();
  points.clear();
  labels.clear();
  images.clear();

  std::vector<int32_t> indexesBefore;
  for (auto it = pointsCache_.begin(); it != pointsCache_.end(); ++it) indexesBefore.push_back(it->first);
  std::vector<int32_t> indexesAfter;

  uint32_t scansRead = 0;

  indexes = tiles_[tileIdxToOffset(i, j)].indexes;
  std::sort(indexes.begin(), indexes.end());
  for (uint32_t t : indexes) {
    indexesAfter.push_back(t);
    if (pointsCache_.find(t) == pointsCache_.end()) {
      scansRead += 1;

      points.push_back(std::shared_ptr<Laserscan>(new Laserscan));
      readPoints(pointclouds_filenames_[t], *points.back());
      pointsCache_[t] = points.back();
      if (defaultPoses) {
        points.back()->pose = Eigen::Matrix4f::Identity();
      } else {
        points.back()->pose = poses_[t];
      }

      labels.push_back(std::shared_ptr<std::vector<uint32_t>>(new std::vector<uint32_t>()));
      readLabels(label_filenames_[t], *labels.back());
      labelCache_[t] = labels.back();

      if (points.back()->size() != labels.back()->size()) {
        std::cout << "Filename: " << pointclouds_filenames_[t] << std::endl;
        std::cout << "Filename: " << label_filenames_[t] << std::endl;
        std::cout << "num. points = " << points.back()->size() << " vs. num. labels = " << labels.back()->size()
                  << std::endl;
        throw std::runtime_error("Inconsistent number of labels.");
      }

    } else {
      points.push_back(pointsCache_[t]);
      labels.push_back(labelCache_[t]);
    }

    images.push_back(image_filenames_[t]);
  }

  std::cout << scansRead << " point clouds read." << std::endl;

  // FIXME: keep more scans in cache. not only remove unloaded scans.

  std::sort(indexesBefore.begin(), indexesBefore.end());
  std::sort(indexesAfter.begin(), indexesAfter.end());

  std::vector<int32_t> needsDelete(indexesBefore.size());
  std::vector<int32_t>::iterator end = std::set_difference(
      indexesBefore.begin(), indexesBefore.end(), indexesAfter.begin(), indexesAfter.end(), needsDelete.begin());

  for (auto it = needsDelete.begin(); it != end; ++it) {
    pointsCache_.erase(*it);
    labelCache_.erase(*it);
  }
}

const KittiReader::Tile& KittiReader::getTile(const Eigen::Vector3f& position) const {
  Eigen::Vector2f idx((position.x() + offset_.x()) / tileSize_, (position.y() + offset_.y()) / tileSize_);
  return tiles_[tileIdxToOffset(idx.x(), idx.y())];
}
const KittiReader::Tile& KittiReader::getTile(uint32_t i, uint32_t j) const { return tiles_[tileIdxToOffset(i, j)]; }

void KittiReader::setTileSize(float size) { tileSize_ = size; }

void KittiReader::update(const std::vector<uint32_t>& indexes, std::vector<LabelsPtr>& labels) {
  for (uint32_t i = 0; i < indexes.size(); ++i) {
    if (labels[i]->size() == 0) {
      std::cout << "Warning: 0 labels?" << std::endl;
      continue;
    }

    if (pointsCache_.find(indexes[i]) == pointsCache_.end()) {
      std::cout << "Warning: labels of non cached points?" << std::endl;
      continue;
    }

    if (labels[i]->size() != pointsCache_[indexes[i]]->size()) {
      std::cout << "Warning: inconsistent numbers of labels for given point cloud!" << std::endl;

      continue;
    }

    if (label_filenames_.size() < indexes[i]) {
      std::cout << "Warning: wrong index?" << std::endl;

      continue;
    }

    std::ofstream out(label_filenames_[indexes[i]].c_str());
    out.write((const char*)&(*labels[i])[0], labels[i]->size() * sizeof(uint32_t));
    out.close();
  }
}

// TODO Änderung  ply lesen
void KittiReader::readPoints(const std::string& filename, Laserscan& scan) {
  std::ifstream in(filename.c_str(), std::ifstream::in);
  if (!in.is_open()) return;

  scan.clear();

  int len = 0;
  std::string currentLine;
  while (std::getline(in, currentLine)) {
    len++;
  }
  in.clear();
  in.seekg(0);

  if (len < 3) {
    std::cout << "Invalid file. Less than 3 lines?" << std::endl;
    return;
  }

  std::getline(in, currentLine);

  if (currentLine.compare("ply") != 0) {
    std::cout << "Invalid file. First line must be \"ply\". Found \"" << currentLine << "\"." << std::endl;
    return;
  }
  std::getline(in, currentLine);

  if (currentLine.compare(0, 7, "format ") != 0) {
    std::cout << "Invalid file. Must contain a format on the second line." << std::endl;
    return;
  }

  if (currentLine.length() < 16 || currentLine.compare(7, 9, "ascii 1.0") != 0) {
    //std::cout << "Using a format which is NOT ascii 1.0!" << std::endl;

    // TODO TONY
    std::vector<Point3f>& points = scan.points;
    std::vector<uint32_t>& colors = scan.colors;
    uint32_t num_points = 0;

    happly::PLYData plyIn(filename, false);
    plyIn.validate();

    std::vector<std::array<double, 3>> vPos = plyIn.getVertexPositions();
    std::vector<std::array<unsigned char, 3>> vCol = plyIn.getVertexColors();

    num_points = vPos.size();

    for(int i = 0; i < num_points; ++i) {
      auto [x, y, z] = vPos[i];
      auto [r, g, b] = vCol[i];

      points.push_back(Point3f(x, y, z));
      colors.push_back(((r<< 16) + (g << 8) + b));
    }


    in.close();

    if (num_points == 0) {
      std::cout << "Warning: No valid points found." << std::endl;
    } else {
      std::cout << "Found " << num_points << " points." << std::endl;
    }

    // std::cout << "Invalid format. Only accepting \"ascii 1.0\", but found \"" << currentLine.substr(7) << "\"." <<
    // std::endl;
    return;
  }
  len -= 2;

  while (len > 0) {
    std::getline(in, currentLine);
    len--;
    if (currentLine.compare("end_header") == 0) {
      break;
    }
  }

  if (len <= 0) {
    std::cout << "Could not find end of header. (\"end_header\" not found)" << std::endl;
    return;
  }
  // TODO TONY
  std::vector<Point3f>& points = scan.points;
  std::vector<uint32_t>& colors = scan.colors;
  uint32_t num_points = 0;
  while (len > 1) {
    std::getline(in, currentLine);
    len--;
    std::istringstream iss(currentLine);
    std::vector<std::string> results((std::istream_iterator<std::string>(iss)), std::istream_iterator<std::string>());
    if (results.size() >= 6) {
      points.push_back(Point3f(std::stof(results[0]), std::stof(results[1]), std::stof(results[2])));
      colors.push_back(((std::stoi(results[3]) << 16) + (std::stoi(results[4]) << 8) + std::stoi(results[5])));
      num_points++;
      /* code */
    } else {
      std::cout << "Warning: Not enough arguments found." << std::endl;
    }
  }
  in.close();

  /// string line;
  /*
  while(std::getline(in, line)) {
      process(&line);
      }
  */
  if (num_points == 0) {
    std::cout << "Warning: No valid points found." << std::endl;
  } else {
    std::cout << "Found " << num_points << " points." << std::endl;
  }
  /*
  uint32_t num_points = len / (4 * sizeof(float));

  std::vector<float> values(4 * num_points);
  in.read((char*)&values[0], 4 * num_points * sizeof(float));

  in.close();
  std::vector<Point3f>& points = scan.points;
  std::vector<float>& colors = scan.colors;

  points.resize(num_points);
  colors.resize(num_points);

  for (uint32_t i = 0; i < num_points; ++i) {
    points.push_back(std::shared_ptr<Point3f>(new Point3f));
    points[i].x = values[4 * i];
    points[i].y = values[4 * i + 1];
    points[i].z = values[4 * i + 2];
    colors[i] = values[4 * i + 3];
  }*/
}

void KittiReader::readLabels(const std::string& filename, std::vector<uint32_t>& labels) {
  std::ifstream in(filename.c_str(), std::ios::binary);
  if (!in.is_open()) {
    std::cerr << "Unable to open label file. " << std::endl;
    return;
  }

  labels.clear();

  in.seekg(0, std::ios::end);
  uint32_t num_points = in.tellg() / (sizeof(uint32_t));
  in.seekg(0, std::ios::beg);

  labels.resize(num_points);
  in.read((char*)&labels[0], num_points * sizeof(uint32_t));

  in.close();
}

void KittiReader::readPoses(const std::string& filename, std::vector<Eigen::Matrix4f>& poses) {
  poses = KITTI::Odometry::loadPoses(filename);

  // convert from camera to pointclouds coordinate system.
  Eigen::Matrix4f Tr = calib_["Tr"];
  Eigen::Matrix4f Tr_inv = Tr.inverse();
  for (uint32_t i = 0; i < poses.size(); ++i) {
    poses[i] = Tr_inv * poses[i] * Tr;
  }
}
