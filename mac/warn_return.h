/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020 Qorvo US, Inc.
 *
 * This software is provided under the GNU General Public License, version 2
 * (GPLv2), as well as under a Qorvo commercial license.
 *
 * You may choose to use this software under the terms of the GPLv2 License,
 * version 2 ("GPLv2"), as published by the Free Software Foundation.
 * You should have received a copy of the GPLv2 along with this program.  If
 * not, see <http://www.gnu.org/licenses/>.
 *
 * This program is distributed under the GPLv2 in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GPLv2 for more
 * details.
 *
 * If you cannot meet the requirements of the GPLv2, you may not use this
 * software for any purpose without first obtaining a commercial license from
 * Qorvo.
 * Please contact Qorvo to inquire about licensing terms.
 *
 * 802.15.4 mac common part sublayer, tools for impossible conditions.
 *
 */

#ifndef WARN_RETURN_H
#define WARN_RETURN_H

#define WARN_RETURN(r)              \
	do {                        \
		if (WARN_ON(r))     \
			return (r); \
	} while (0)

#define WARN_RETURN_NULL(r)          \
	do {                         \
		if (WARN_ON(r))      \
			return NULL; \
	} while (0)

#define WARN_UNREACHABLE_DEFAULT() WARN(1, "unreachable case")

#endif /* WARN_RETURN_H */
