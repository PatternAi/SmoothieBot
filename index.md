---
layout: home
title: Home
nav_order: 0
description: >-
    Just the Class is a modern, highly customizable, responsive Jekyll theme
    for developing course websites.
---

# Smoothie Bot
{: .mb-2 }
EECS 106A, Fall 2019 
{: .mb-0 .fs-6 .text-grey-dk-000 }

{% assign instructors = site.staffers | where: 'role', 'Project Group' %}
<div class="role">
  {% for staffer in instructors %}
  {{ staffer }}
  {% endfor %}
</div>

{% if site.announcements %}
{{ site.announcements.last }}
<a href="{{ site.baseurl }}/announcements" class="btn btn-outline fs-3">
  All Announcements
</a>
{% endif %}

## Smoothie Bot

Picture this: It’s a hot, summer afternoon, you’ve just finished playing soccer with your kid. The heat haswithered away your energy, and the thirst in your mouth is quite unbearable. The thought pops up in yourconscience:Damn, a smoothie sounds sooo freaking good right about now, but Jamba Juice is oh so far!Enter, your own personal SmoothieBot - capable of freshly blending any smoothie of your heart’s desire,right in the comfort of your home - with no effort required!
