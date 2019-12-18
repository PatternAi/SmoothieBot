---
layout: page
title: Team
nav_order: 6
description: A listing of project members
---

# Team Members
---
<div class="role">
  {% assign instructors = site.staffers | where: 'role', 'Team Member' %}
  {% for staffer in instructors %}
  {{ staffer }}
  {% endfor %}
</div>

<img class="one" src="../pictures/team.jpg" alt="Team Photo"/>
