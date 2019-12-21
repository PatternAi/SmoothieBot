---
layout: page
title: Introduction
nav_order: 1
description: >-
    Overview of smoothiebot project
---

# Introduction
{:.no_toc}

## Table of contents
{: .no_toc .text-delta }

1. Goal
2. Technical Motivation
3. Real-World Impact
{:toc}

---

## Goal

The end goal of our project was to implement a program that allows Baxter to assemble a smoothie according to ingredients presented by the user. SmoothieBot, as we affectionately call it, is able to recognize fruit on a table, cross-reference those available fruit with the recipe, move the selected fruit to the blender, and pick up and close the lid once the recipe is fulfilled.

## Technical Motivation
Our project tackles a unique vision problem with an innovative, lightweight solution that doesn’t rely on any complex machine learning. We took advantage of the narrowness of our classification task, classifying fruit, and realized that we only needed to consider two main features to distinguish fruit: shape and color. With that in mind, we sought to develop a creative classification solution that ingests, filters, and clusters point cloud data and classifies based on these two fruit characteristics. In addition to classifying fruit, the task of grabbing fruit presented a difficult challenge due to the wide variety in fruit shapes. Picking up any fruit in any orientation required clever use of point cloud analysis and a robust path planning algorithm.

## Real-World Impact
The use of robotics in the food industry as waiters and chefs are gradually becoming more mainstream. Companies such as CafeX use 6-axle robots in its cafes to serve as the sole barista. Our work could be applied to detecting, grasping, and placing ingredients in a kitchen setting.
