import LaunchEditor from './launch-editor.svelte';
import type { SubsystemInfo, Node } from '$lib/cli-bindings';

export type LaunchEditorProps = {
	subsystems: SubsystemInfo[];
	activeNodes: Node[];
	onNodeChanged: (node: Node, selected: boolean) => void;
};

export { LaunchEditor };
