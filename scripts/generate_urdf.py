#!/usr/bin/env python

#
# Runs on Python version 2.7
#
import os
import sys
import shutil
import xml.etree.ElementTree as ET
import subprocess

def update_urdf(urdf_filepath):
  tree = ET.parse(urdf_filepath)
  root = tree.getroot()

  for i in root.iter():
    if i.tag == 'robot':
      i.set('name', 'mc_aliengo_description')

  for link in root.findall('link'):
    if not 'name' in link.attrib:
      continue
    if link.attrib['name'] == 'base' or link.attrib['name'] == 'imu_link':
      continue
    
    # update visual
    for visual in link.findall('visual'):
      for i in visual.iter():
        if i.tag == 'origin':
          origin_attrib = i.attrib
        elif i.tag == 'mesh':
          i.set('filename', i.attrib['filename'].replace('aliengo_description', 'mc_aliengo_description'))
          mesh_attrib = i.attrib

    # update collision
    for collision in link.findall('collision'):
      # change origin
      for origin in collision.findall('origin'):
        origin.set('rpy', origin_attrib['rpy'])
        origin.set('xyz', origin_attrib['xyz'])

      # remove geometry
      for geometry in collision.findall('geometry'):
        collision.remove(geometry)

      # add mash
      sub_geometry = ET.SubElement(collision, 'geometry')
      ET.SubElement(sub_geometry, 'mesh', mesh_attrib)

  # save
  tree.write(urdf_filepath, encoding="utf-8", xml_declaration=True)

  # reformat/reindent
  subprocess.call(['xmllint', '--format', urdf_filepath, '-o', urdf_filepath])


if __name__ == '__main__':
  args = sys.argv
  if 2 != len(args):
    print("Usage: python ./generate_urdf.py aliengo_description_urdf_filepath")
    exit()

  # copy original urdf to /tmp
  org_urdf_filepath = args[1]
  work_urdf_path = '/tmp/urdf'
  work_urdf_filepath = work_urdf_path + '/aliengo.urdf'
  if not os.path.isdir(work_urdf_path):
    os.mkdir(work_urdf_path)
  shutil.copyfile(org_urdf_filepath, work_urdf_filepath)

  # update urdf
  update_urdf(work_urdf_filepath)

  # move update file
  mc_aliengo_description_path = '../'
  if os.path.isdir(mc_aliengo_description_path + 'urdf'):
    shutil.move(work_urdf_filepath, mc_aliengo_description_path + 'urdf/aliengo.urdf')
  else:
    shutil.move(work_urdf_path, mc_aliengo_description_path)

