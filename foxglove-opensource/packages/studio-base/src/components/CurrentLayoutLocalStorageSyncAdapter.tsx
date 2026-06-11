// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/

import assert from "assert";
import { useEffect } from "react";
import { useDebounce } from "use-debounce";

import Log from "@foxglove/log";
import { LOCAL_STORAGE_STUDIO_LAYOUT_KEY } from "@foxglove/studio-base/constants/localStorageKeys";
import {
  LayoutState,
  useCurrentLayoutActions,
  useCurrentLayoutSelector,
} from "@foxglove/studio-base/context/CurrentLayoutContext";
import { LayoutData } from "@foxglove/studio-base/context/CurrentLayoutContext/actions";
import { usePlayerSelection } from "@foxglove/studio-base/context/PlayerSelectionContext";
import { defaultLayout } from "@foxglove/studio-base/providers/CurrentLayoutProvider/defaultLayout";
import { migratePanelsState } from "@foxglove/studio-base/services/migrateLayout";

function selectLayoutData(state: LayoutState) {
  return state.selectedLayout?.data;
}

const log = Log.getLogger(__filename);

function migrateRslidarWebTopic(layoutData: LayoutData): LayoutData {
  const configById = { ...layoutData.configById };
  let changed = false;

  for (const [panelId, config] of Object.entries(configById)) {
    const maybeConfig = config as
      | {
          followTf?: string;
          topics?: Record<string, unknown>;
        }
      | undefined;
    const topics = maybeConfig?.topics;
    if (topics == undefined || !("/rslidar_points" in topics)) {
      continue;
    }

    const oldTopicConfig = topics["/rslidar_points"];
    const migratedTopics = { ...topics };
    migratedTopics["/rslidar_points_web"] = {
      ...(oldTopicConfig != undefined && typeof oldTopicConfig === "object" ? oldTopicConfig : {}),
      visible: true,
      pointSize: 3,
      colorMode: "flat",
      flatColor: "#00ff66",
    };
    delete migratedTopics["/rslidar_points"];

    configById[panelId] = {
      ...(maybeConfig as object),
      followTf: maybeConfig?.followTf ?? "rslidar",
      topics: migratedTopics,
    };
    changed = true;
  }

  return changed ? { ...layoutData, configById } : layoutData;
}

export function CurrentLayoutLocalStorageSyncAdapter(): JSX.Element {
  const { selectedSource } = usePlayerSelection();

  const { setCurrentLayout } = useCurrentLayoutActions();
  const currentLayoutData = useCurrentLayoutSelector(selectLayoutData);

  useEffect(() => {
    if (selectedSource?.sampleLayout) {
      setCurrentLayout({ data: selectedSource.sampleLayout });
    }
  }, [selectedSource, setCurrentLayout]);

  const [debouncedLayoutData] = useDebounce(currentLayoutData, 250, { maxWait: 500 });

  useEffect(() => {
    if (!debouncedLayoutData) {
      return;
    }

    const serializedLayoutData = JSON.stringify(debouncedLayoutData);
    assert(serializedLayoutData);
    localStorage.setItem(LOCAL_STORAGE_STUDIO_LAYOUT_KEY, serializedLayoutData);
  }, [debouncedLayoutData]);

  useEffect(() => {
    log.debug(`Reading layout from local storage: ${LOCAL_STORAGE_STUDIO_LAYOUT_KEY}`);

    const serializedLayoutData = localStorage.getItem(LOCAL_STORAGE_STUDIO_LAYOUT_KEY);

    if (serializedLayoutData) {
      log.debug("Restoring layout from local storage");
    } else {
      log.debug("No layout found in local storage. Using default layout.");
    }

    const layoutData = migrateRslidarWebTopic(
      migratePanelsState(
        serializedLayoutData ? (JSON.parse(serializedLayoutData) as LayoutData) : defaultLayout,
      ),
    );
    setCurrentLayout({ data: layoutData });
  }, [setCurrentLayout]);

  return <></>;
}
